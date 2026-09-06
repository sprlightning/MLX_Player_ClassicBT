/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"

/* IDF LHDC V5 解码器配置入口（external 集成层导出）：流配置事件后重试 workspace 分配 */
extern void a2dp_lhdcv5_decoder_configure(const uint8_t *p_codec_info);

#include "bt_app_core.h" // not use it's i2s function, use bt_app_work_dispatch function and codec change msg queue function
#include "bt_app_av.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
// #include "esp_a2dp_api.h"
#include "codec_config.h"
// A2DP_LHDCV5_CODEC_LEN（codec_info 布局见 a2dp_vendor_lhdcv5_constants.h 头注释）
#include "codec_config/a2dp_vendor_lhdc_constants.h"
#include "esp_avrc_api.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_pm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/semphr.h"
#include "task_config.h"

#ifndef LOCAL_DEVICE_NAME
#define LOCAL_DEVICE_NAME CONFIG_LOCAL_DEVICE_NAME
#endif

#include "audio_output_i2s.h"

#include "sys/lock.h"

/* background work queue: offloads NVS/I2S/DAC/reconnect from BtAppTask */
#define BT_AV_BG_QUEUE_LEN              (20)

typedef enum {
    BT_AV_BG_WORK_NONE = 0,
    BT_AV_BG_WORK_I2S_STOP,
    BT_AV_BG_WORK_I2S_START,
    BT_AV_BG_WORK_AUDIO_CFG,
    BT_AV_BG_WORK_SAVE_BDA,
    BT_AV_BG_WORK_RECONNECT,
} bt_av_bg_work_type_t;

typedef struct {
    bt_av_bg_work_type_t type;
    uint8_t bda[ESP_BD_ADDR_LEN];
    uint32_t sample_rate;
    uint8_t bit_depth;
    uint8_t channels;
} bt_av_bg_work_t;

/* AVRCP used transaction labels */
#define APP_RC_CT_TL_GET_CAPS            (0)
#define APP_RC_CT_TL_GET_META_DATA       (1)
#define APP_RC_CT_TL_RN_TRACK_CHANGE     (2)
#define APP_RC_CT_TL_RN_PLAYBACK_CHANGE  (3)
#define APP_RC_CT_TL_RN_PLAY_POS_CHANGE  (4)

/* Application layer causes delay value */
#define APP_DELAY_VALUE                  50  // 5ms

/* NVS keys for the last connected peer address, aligned with ESP32-A2DP */
#define A2DP_LAST_BDA_NAMESPACE          "connected_bda"
#define A2DP_LAST_BDA_KEY                "last_bda"

/* NVS keys for BT settings (auto reconnect switch) */
#define BT_SETTINGS_NAMESPACE            "bt_settings"
#define BT_SETTINGS_AUTO_RECONNECT_KEY   "auto_reconnect"

/* Reconnect parameters: align with ESP32-A2DP which reconnects immediately.
   A short delay (300-500 ms) lets the phone tear down the old AVRCP control
   channel while still winning the reconnect race. */
#define A2DP_RECONNECT_DELAY_MS          500
#define A2DP_RECONNECT_MAX_RETRY         5

/*******************************
 * STATIC FUNCTION DECLARATIONS
 ******************************/

/* allocate new meta buffer */
static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param);
/* handler for new track is loaded */
static void bt_av_new_track(void);
/* handler for track status change */
static void bt_av_playback_changed(void);
/* handler for track playing position change */
static void bt_av_play_pos_changed(void);
/* notification event handler */
static void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter);
/* installation for i2s */
static void bt_i2s_driver_install(void);
/* uninstallation for i2s */
static void bt_i2s_driver_uninstall(void);
/* save/read last connected peer address from NVS */
static void bt_a2dp_save_last_bda(const uint8_t *bda);
static void bt_a2dp_read_last_bda(uint8_t *bda);
/* apply audio cfg to I2S and DAC (starts I2S) */
static void bt_a2dp_do_apply_audio_cfg(uint32_t sample_rate, uint8_t bit_depth, uint8_t channels);
/* convert internal codec_type_t to human-readable string */
static const char* codec_type_to_string(codec_type_t codec_type);
/* request apply of cached cfg (used in CONNECTED/STARTED fallback) */
static void bt_a2dp_apply_cached_audio_cfg(void);
/* background task and helpers to offload NVS/I2S/DAC/reconnect */
static void bt_av_bg_task(void *arg);
static bool bt_av_bg_send_work(const bt_av_bg_work_t *work);
static void bt_av_bg_enqueue_i2s_stop(void);
static void bt_av_bg_enqueue_i2s_start(void);
static void bt_av_bg_enqueue_audio_cfg(uint32_t sample_rate, uint8_t bit_depth, uint8_t channels);
static void bt_av_bg_enqueue_save_bda(const uint8_t *bda);
static void bt_av_bg_enqueue_reconnect(void);
/* one-shot reconnect task */
static void bt_a2dp_reconnect_task(void *arg);
/* set volume by remote controller */
static void volume_set_by_controller(uint8_t volume);
/* set volume by local host */
static void volume_set_by_local_host(uint8_t volume);
/* simulation volume change */
/* a2dp event handler */
static void bt_av_hdl_a2d_evt(uint16_t event, void *p_param);
/* avrc controller event handler */
static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param);
/* avrc target event handler */
static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param);

/* bluetooth stack enabled event */
enum {
    BT_APP_EVT_STACK_UP = 0,
};
void bt_av_hdl_stack_evt(uint16_t event, void *p_param);
static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);

/*******************************
 * STATIC VARIABLE DEFINITIONS
 ******************************/

static uint32_t s_pkt_cnt = 0;               /* count for audio packet */
static esp_a2d_audio_state_t s_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;
                                             /* audio stream datapath state */
static const char *s_a2d_conn_state_str[] = {"Disconnected", "Connecting", "Connected", "Disconnecting"};
                                             /* connection state in string */
static const char *s_a2d_audio_state_str[] = {"Suspended", "Stopped", "Started"};
                                             /* audio stream datapath state in string */
static esp_avrc_rn_evt_cap_mask_t s_avrc_peer_rn_cap;
                                             /* AVRC target notification capability bit mask */
static _lock_t s_volume_lock;
static uint8_t s_volume = DEFAULT_SYSTEM_VOLUME; /* 本地系统音量（AVRCP absolute volume 同级，0-127） */
static bool s_volume_notify;                 /* notify volume change or not */
static uint8_t s_peer_bda[ESP_BD_ADDR_LEN] = {0}; /* last connected peer bd address */
static bool s_a2dp_connected = false;        /* a2dp connection state */
static bool s_a2dp_connecting = false;       /* a2dp 正在建立连接/协商 codec（CONNECTING 状态） */
static bool s_a2dp_playing = false;          /* a2dp audio started */
/* Bluedroid BTM 功率管理：连接后保持链路 ACTIVE（禁用 sniff），避免射频低活跃
 * 拖慢 A2DP/AVRCP 协商与响应（esp-idf 无官方 API，直接 extern stack 函数） */
typedef struct {
    uint16_t min;
    uint16_t max;
    uint16_t attempt;
    uint16_t timeout;
    uint8_t  mode;
} bt_btm_pm_md_t;
extern uint8_t BTM_SetPowerMode(uint8_t pm_id, uint8_t *remote_bda, bt_btm_pm_md_t *p_mode);
#define BT_BTM_PM_SET_ONLY_ID 0x80
#define BT_BTM_PM_MD_ACTIVE   0

static void bt_av_keep_active(const uint8_t *bda)
{
    bt_btm_pm_md_t md = { 0, 0, 0, 0, BT_BTM_PM_MD_ACTIVE };
    uint8_t ret = BTM_SetPowerMode(BT_BTM_PM_SET_ONLY_ID, (uint8_t *)bda, &md);
    ESP_LOGI(BT_AV_TAG, "BTM keep active (disable sniff) ret=%u", ret);
}
/* A2DP 断开完成信号量：主动断开时等待该信号量（DISCONNECTED 回调
 * give），确保 bluedroid 内部状态机完成迁移后再 deinit profile，避免播放中 deinit
 * 触发 btc_a2dp_sink_shutdown 对已释放媒体线程 post 崩溃。 */
static SemaphoreHandle_t s_a2dp_disc_sem = NULL;
#ifdef CONFIG_PM_ENABLE
static esp_pm_lock_handle_t s_bt_pm_lock = NULL; /* 蓝牙运行时锁定 CPU 频率，防止 PM 导致看门狗复位 */
#endif
static codec_type_t s_codec_type = CODEC_UNKNOWN; /* current a2dp codec */
static uint32_t s_last_cfg_ms = 0;   /* 最近一次 codec 配置时刻（补位延时基准） */
static char s_title[256] = {0};              /* avrcp title（实际为歌词） */
static char s_song_title[256] = {0};         /* 歌曲标题：取 avrcp artist（部分手机把歌名放在 artist） */
static uint32_t s_play_pos_ms = 0;           /* avrcp 播放位置（AVRC 通知） */
static uint8_t s_current_track_uid[8] = {0};  /* 当前曲目 UID，用于过滤重复/无效的曲目变更通知 */

/* cached audio cfg, used when ESP_A2D_AUDIO_CFG_EVT is missing */
static uint32_t s_last_sample_rate = 0;
static uint8_t  s_last_bit_depth   = 0;
static uint8_t  s_last_channels    = 0;
static bool     s_last_cfg_valid   = false;
/* true if an AUDIO_CFG_EVT has been received since the last CONNECTED */
static bool     s_audio_cfg_received_after_connect = false;

/* background work queue/task to offload NVS/I2S/DAC/reconnect */
static QueueHandle_t s_bt_av_bg_queue = NULL;
static TaskHandle_t s_bt_av_bg_task_hdl = NULL;
/* guard against concurrent reconnect tasks */
static bool s_reconnect_task_running = false;


/********************************
 * STATIC FUNCTION DEFINITIONS
 *******************************/

static void bt_app_alloc_meta_buffer(esp_avrc_ct_cb_param_t *param)
{
    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(param);
    uint8_t *attr_text = (uint8_t *) malloc (rc->meta_rsp.attr_length + 1);

    memcpy(attr_text, rc->meta_rsp.attr_text, rc->meta_rsp.attr_length);
    attr_text[rc->meta_rsp.attr_length] = 0;
    rc->meta_rsp.attr_text = attr_text;
}

static void bt_av_new_track(void)
{
    /* request metadata */
    uint8_t attr_mask = ESP_AVRC_MD_ATTR_TITLE |
                        ESP_AVRC_MD_ATTR_ARTIST |
                        ESP_AVRC_MD_ATTR_ALBUM |
                        ESP_AVRC_MD_ATTR_GENRE;
    esp_avrc_ct_send_metadata_cmd(APP_RC_CT_TL_GET_META_DATA, attr_mask);

    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_TRACK_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_TRACK_CHANGE,
                                                   ESP_AVRC_RN_TRACK_CHANGE, 0);
    }
}

static void bt_av_playback_changed(void)
{
    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_PLAY_STATUS_CHANGE)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAYBACK_CHANGE,
                                                   ESP_AVRC_RN_PLAY_STATUS_CHANGE, 0);
    }
}

static void bt_av_play_pos_changed(void)
{
    /* register notification if peer support the event_id */
    if (esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_TEST, &s_avrc_peer_rn_cap,
                                           ESP_AVRC_RN_PLAY_POS_CHANGED)) {
        esp_avrc_ct_send_register_notification_cmd(APP_RC_CT_TL_RN_PLAY_POS_CHANGE,
                                                   ESP_AVRC_RN_PLAY_POS_CHANGED, 10);
    }
}

static void bt_av_notify_evt_handler(uint8_t event_id, esp_avrc_rn_param_t *event_parameter)
{
    switch (event_id) {
    /* when new track is loaded, this event comes */
    case ESP_AVRC_RN_TRACK_CHANGE:
        /* 仅当曲目 UID 真正变化时才清空旧标题/歌词，
           避免部分手机把歌词/元数据更新也当作曲目切换。
           无论 UID 是否变化都需要重新注册 TRACK_CHANGE 通知。 */
        if (memcmp(event_parameter->elm_id, s_current_track_uid, 8) != 0) {
            memcpy(s_current_track_uid, event_parameter->elm_id, 8);
            s_song_title[0] = '\0';
            s_title[0] = '\0';
        }
        bt_av_new_track();
        break;
    /* when track status changed, this event comes */
    case ESP_AVRC_RN_PLAY_STATUS_CHANGE:
        ESP_LOGI(BT_AV_TAG, "Playback status changed: 0x%x", event_parameter->playback);
        /* 用 AVRCP 推送的播放状态校正 s_a2dp_playing（重连后手机端状态同步）：
         * PLAYING=1 → true，STOPPED/PAUSED 等 → false。配合 disconnect/断开时
         * 的复位，保证 esp32 播放状态与手机端一致，按播放键发正确的 PLAY/PAUSE。 */
        s_a2dp_playing = (event_parameter->playback == ESP_AVRC_PLAYBACK_PLAYING);
        bt_av_playback_changed();
        break;
    /* when track playing position changed, this event comes */
    case ESP_AVRC_RN_PLAY_POS_CHANGED:
        ESP_LOGD(BT_AV_TAG, "Play position changed: %"PRIu32"-ms", event_parameter->play_pos);  // test OK, value saved to s_play_pos_ms
        s_play_pos_ms = event_parameter->play_pos;
        bt_av_play_pos_changed();
        break;
    /* others */
    default:
        ESP_LOGI(BT_AV_TAG, "unhandled event: %d", event_id);
        break;
    }
}

void bt_i2s_driver_install(void)
{
    /* I2S 已在 main 中初始化，这里只需启动 */
    audio_output_i2s_start();
}

static void __attribute__((unused)) bt_i2s_driver_uninstall(void)
{
    audio_output_i2s_stop();
}

/* one-shot task to reconnect a2dp after disconnection, to restore AVRCP.
   Kept as a dedicated task so the background queue is not blocked by the
   reconnect delay/backoff. The task self-destructs when done. */
static void bt_a2dp_reconnect_task(void *arg)
{
    (void) arg;
    uint8_t last_bda[ESP_BD_ADDR_LEN] = {0};
    bt_a2dp_read_last_bda(last_bda);

    if (memcmp(last_bda, "\0\0\0\0\0\0", ESP_BD_ADDR_LEN) == 0) {
        s_reconnect_task_running = false;
        vTaskDelete(NULL);
    }

    /* 循环重试：首次 connect 常因对端未就绪（page scan 窗口/刚开机）而失败
     * （实测 SDP conn error 0x9 + BTA_AV_OPEN_EVT FAILED status 2，失败断开被
     * 报为 NORMAL 不会再次入队）——单次尝试即放弃会表现为"开机不回连"。
     * 每次发起后等待结果窗口（连上即退出），失败按递增间隔重试直到上限。 */
    for (int retry = 1; retry <= A2DP_RECONNECT_MAX_RETRY && !s_a2dp_connected; retry++) {
        vTaskDelay(pdMS_TO_TICKS(A2DP_RECONNECT_DELAY_MS + (retry - 1) * 1500));
        if (s_a2dp_connected) {
            break;
        }
        ESP_LOGI(BT_AV_TAG, "A2DP reconnecting to last device (retry %d/%d)",
                 retry, A2DP_RECONNECT_MAX_RETRY);
        esp_err_t cret = esp_a2d_sink_connect(last_bda);
        if (cret != ESP_OK) {
            ESP_LOGW(BT_AV_TAG, "esp_a2d_sink_connect returned: %s", esp_err_to_name(cret));
        }
        /* 等待连接结果窗口：连上（CONNECTED 事件置位）即退出；超时则下一轮重试 */
        for (int w = 0; w < 40 && !s_a2dp_connected; w++) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
    if (!s_a2dp_connected) {
        ESP_LOGW(BT_AV_TAG, "A2DP reconnect retry limit reached, stop reconnecting");
    }

    s_reconnect_task_running = false;
    vTaskDelete(NULL);
}

/* background task: applies I2S/DAC reconfiguration, NVS writes, etc.
   This keeps BtAppTask responsive so that ESP_A2D_AUDIO_CFG_EVT and AVRCP
   events are not dropped when the phone switches LHDC V5 configuration. */
static void bt_av_bg_task(void *arg)
{
    (void) arg;
    bt_av_bg_work_t work;

    for (;;) {
        if (xQueueReceive(s_bt_av_bg_queue, &work, portMAX_DELAY) == pdTRUE) {
            switch (work.type) {
            case BT_AV_BG_WORK_I2S_STOP:
                audio_output_i2s_stop();
                break;
            case BT_AV_BG_WORK_I2S_START:
                audio_output_i2s_start();
                break;
            case BT_AV_BG_WORK_AUDIO_CFG:
                bt_a2dp_do_apply_audio_cfg(work.sample_rate, work.bit_depth, work.channels);
                break;
            case BT_AV_BG_WORK_SAVE_BDA:
                bt_a2dp_save_last_bda(work.bda);
                break;
            case BT_AV_BG_WORK_RECONNECT:
                /* Reconnect is handled by a dedicated task so the queue can
                   continue processing other work while waiting. */
                if (!s_reconnect_task_running) {
                    s_reconnect_task_running = true;
                    if (xTaskCreatePinnedToCore(bt_a2dp_reconnect_task, "bt_reconnect", BT_RECONNECT_STACK_SIZE, NULL,
                                                BT_RECONNECT_PRIORITY, NULL, BT_RECONNECT_CORE) != pdPASS) {
                        s_reconnect_task_running = false;
                    }
                }
                break;
            default:
                break;
            }
        }
    }
}

static bool bt_av_bg_send_work(const bt_av_bg_work_t *work)
{
    if (s_bt_av_bg_queue == NULL) {
        s_bt_av_bg_queue = xQueueCreate(BT_AV_BG_QUEUE_LEN, sizeof(bt_av_bg_work_t));
    }
    if (s_bt_av_bg_task_hdl == NULL && s_bt_av_bg_queue != NULL) {
        /* Pin background work task to Core 0 so I2S/DAC reconfig does not compete
           with the LHDC decoder running on Core 1. */
        xTaskCreatePinnedToCore(bt_av_bg_task, "BtAvBgTask", BT_AV_BG_STACK_SIZE, NULL, BT_AV_BG_PRIORITY,
                                &s_bt_av_bg_task_hdl, BT_AV_BG_CORE);
    }
    if (s_bt_av_bg_queue == NULL) {
        return false;
    }
    if (xQueueSend(s_bt_av_bg_queue, work, 100 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGW(BT_AV_TAG, "Background work queue full, dropped work %d", work->type);
        return false;
    }
    return true;
}

static void bt_av_bg_enqueue_i2s_stop(void)
{
    bt_av_bg_work_t work = {.type = BT_AV_BG_WORK_I2S_STOP};
    bt_av_bg_send_work(&work);
}

static void __attribute__((unused)) bt_av_bg_enqueue_i2s_start(void)
{
    bt_av_bg_work_t work = {.type = BT_AV_BG_WORK_I2S_START};
    bt_av_bg_send_work(&work);
}

static void bt_av_bg_enqueue_audio_cfg(uint32_t sample_rate, uint8_t bit_depth, uint8_t channels)
{
    bt_av_bg_work_t work = {
        .type = BT_AV_BG_WORK_AUDIO_CFG,
        .sample_rate = sample_rate,
        .bit_depth = bit_depth,
        .channels = channels,
    };
    bt_av_bg_send_work(&work);
}

static void bt_av_bg_enqueue_save_bda(const uint8_t *bda)
{
    bt_av_bg_work_t work = {.type = BT_AV_BG_WORK_SAVE_BDA};
    if (bda != NULL) {
        memcpy(work.bda, bda, ESP_BD_ADDR_LEN);
    }
    bt_av_bg_send_work(&work);
}

static void bt_av_bg_enqueue_reconnect(void)
{
    bt_av_bg_work_t work = {.type = BT_AV_BG_WORK_RECONNECT};
    bt_av_bg_send_work(&work);
}


static void volume_set_by_controller(uint8_t volume)
{
    ESP_LOGI(BT_RC_TG_TAG, "Volume is set by remote controller to: %"PRIu32"%%", (uint32_t)volume * 100 / 0x7f);
    /* set the volume in protection of lock */
    _lock_acquire(&s_volume_lock);
    s_volume = volume;
    _lock_release(&s_volume_lock);

    /* 系统音量（0-127）→ audio_output 映射（软件增益，PCM5102A 无硬件音量） */
    audio_output_set_volume(volume);
}

static void volume_set_by_local_host(uint8_t volume)
{
    ESP_LOGI(BT_RC_TG_TAG, "Volume is set locally to: %"PRIu32"%%", (uint32_t)volume * 100 / 0x7f);
    /* set the volume in protection of lock */
    _lock_acquire(&s_volume_lock);
    s_volume = volume;
    _lock_release(&s_volume_lock);

    audio_output_set_volume(volume);

    /* send notification response to remote AVRCP controller */
    if (s_volume_notify) {
        esp_avrc_rn_param_t rn_param;
        rn_param.volume = s_volume;
        esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_CHANGED, &rn_param);
        s_volume_notify = false;
    }
}


/* save last connected peer address to NVS, aligned with ESP32-A2DP */
static void bt_a2dp_save_last_bda(const uint8_t *bda)
{
    if (bda == NULL || memcmp(bda, "\0\0\0\0\0\0", ESP_BD_ADDR_LEN) == 0) {
        return;
    }

    nvs_handle_t nvs_handle;
    if (nvs_open(A2DP_LAST_BDA_NAMESPACE, NVS_READWRITE, &nvs_handle) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "NVS open failed for saving last_bda");
        return;
    }

    if (nvs_set_blob(nvs_handle, A2DP_LAST_BDA_KEY, bda, ESP_BD_ADDR_LEN) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "NVS set last_bda failed");
    }
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
}

/* read last connected peer address from NVS */
static void bt_a2dp_read_last_bda(uint8_t *bda)
{
    if (bda == NULL) {
        return;
    }
    memset(bda, 0, ESP_BD_ADDR_LEN);

    nvs_handle_t nvs_handle;
    if (nvs_open(A2DP_LAST_BDA_NAMESPACE, NVS_READONLY, &nvs_handle) != ESP_OK) {
        ESP_LOGD(BT_AV_TAG, "NVS open failed for reading last_bda");
        return;
    }

    size_t len = ESP_BD_ADDR_LEN;
    esp_err_t err = nvs_get_blob(nvs_handle, A2DP_LAST_BDA_KEY, bda, &len);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(BT_AV_TAG, "NVS get last_bda failed");
    }
    nvs_close(nvs_handle);
}

/* actually apply audio cfg to I2S and DAC; starts I2S before returning.
   Runs in BtAvBgTask context so BtAppTask stays responsive. */
static void bt_a2dp_do_apply_audio_cfg(uint32_t sample_rate, uint8_t bit_depth, uint8_t channels)
{
    if (sample_rate == 0) {
        return;
    }

    /* 仅当 I2S 当前配置与缓存不一致时才动作，避免 CONNECTED/STARTED 时不必要的 stop/start */
    if (audio_output_i2s_get_sample_rate() == sample_rate &&
        audio_output_i2s_get_bit_depth() == bit_depth &&
        audio_output_i2s_get_channels() == channels) {
        if (!audio_output_i2s_get_sample_rate()) {
            return;
        }
        ESP_LOGD(BT_AV_TAG, "Audio cfg already applied, ensure I2S started");
        audio_output_i2s_start();
        return;
    }

    ESP_LOGI(BT_AV_TAG, "Apply audio cfg: %"PRIu32"Hz | %ubit | %uch",
             sample_rate, bit_depth, channels);

    audio_output_i2s_stop();
    audio_output_i2s_configure(sample_rate, bit_depth, channels, I2S_STD_FORMAT);

    /* 配置完成后必须重新启动 I2S，否则后续音频数据无法输出 */
    audio_output_i2s_start();
}

/* request that cached audio cfg be applied asynchronously in BtAvBgTask */
static void bt_a2dp_apply_cached_audio_cfg(void)
{
    if (!s_last_cfg_valid) {
        return;
    }
    bt_av_bg_enqueue_audio_cfg(s_last_sample_rate, s_last_bit_depth, s_last_channels);
}

// 将输入的编码类型解析成字符类型
// 包括SBC、aptX、aptX-LL、aptX-HD、LDAC、OPUS、LC3 Plus、AAC
static const char* codec_type_to_string(codec_type_t codec_type) {
  switch (codec_type) {
    case CODEC_SBC: return "SBC"; break;
    case CODEC_APTX: return "aptX"; break;
    case CODEC_APTX_LL: return "aptX-LL"; break;
    case CODEC_APTX_HD: return "aptX-HD"; break;
    case CODEC_LDAC: return "LDAC"; break;
    case CODEC_OPUS: return "OPUS"; break;
    case CODEC_LC3PLUS: return "LC3 Plus"; break;
    case CODEC_AAC: return "AAC"; break;
    case CODEC_LHDCV5: return "LHDC V5"; break;
    default: return "Unknown";
  }
}

/* 编码缩写（显示用）：
 * aptX-LL -> aptX-L、aptX-HD -> aptX-H、LC3 Plus -> LC3P、LHDC V5 -> LHDCV5 */
static const char* codec_type_to_abbrev(codec_type_t codec_type) {
  switch (codec_type) {
    case CODEC_SBC: return "SBC"; break;
    case CODEC_APTX: return "aptX"; break;
    case CODEC_APTX_LL: return "aptX-L"; break;
    case CODEC_APTX_HD: return "aptX-H"; break;
    case CODEC_LDAC: return "LDAC"; break;
    case CODEC_OPUS: return "OPUS"; break;
    case CODEC_LC3PLUS: return "LC3P"; break;
    case CODEC_AAC: return "AAC"; break;
    case CODEC_LHDCV5: return "LHDCV5"; break;
    default: return "UNK";
  }
}

static void bt_av_hdl_a2d_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    esp_a2d_cb_param_t *a2d = NULL;

    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_A2D_CONNECTION_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        uint8_t *bda = a2d->conn_stat.remote_bda;
        ESP_LOGI(BT_AV_TAG, "A2DP connection state: %s, [%02x:%02x:%02x:%02x:%02x:%02x]",
            s_a2d_conn_state_str[a2d->conn_stat.state], bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
            s_a2dp_connected = false;
            s_a2dp_connecting = false;
            /* 断开即视为停止：复位播放状态，与手机端实际状态保持一致。
             * 否则主动断开后残留 true，回连时状态不同步。 */
            s_a2dp_playing = false;
            s_audio_cfg_received_after_connect = false;
            /* 通知等待断开完成的信号量（若在等）：
             * 断开回调到达时 bluedroid 内部 bta_av 状态机已完成 OPENED→IDLE
             * 迁移，此时再 deinit A2DP profile 才安全。 */
            if (s_a2dp_disc_sem != NULL) {
                xSemaphoreGive(s_a2dp_disc_sem);
            }
            esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
            /* Offload I2S stop/NVS save/reconnect to BtAvBgTask so BtAppTask
               can keep draining the event queue during a codec switch storm. */
            bt_av_bg_enqueue_i2s_stop();
            bt_av_bg_enqueue_save_bda(bda);
            /* 仅在异常断开（远程关闭连接）时自动重连；主动本地断开时 disc_rsn == NORMAL，不重连 */
            if (a2d->conn_stat.disc_rsn != ESP_A2D_DISC_RSN_NORMAL) {
                bt_av_bg_enqueue_reconnect();
            }
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED){
            s_a2dp_connected = true;
            s_a2dp_connecting = false;
            s_audio_cfg_received_after_connect = false;
            memcpy(s_peer_bda, bda, ESP_BD_ADDR_LEN);
            esp_bt_gap_set_scan_mode(ESP_BT_NON_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
            /* 部分 LHDC V5 采样率切换时 ESP_A2D_AUDIO_CFG_EVT 不会上报，
               在连接成功后主动应用缓存配置作为后备（BtAvBgTask 执行） */
            bt_a2dp_apply_cached_audio_cfg();
        } else if (a2d->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTING) {
            s_a2dp_connecting = true;
            bt_i2s_driver_install();
        }
        break;
    }
    /* when audio stream transmission state changed, this event comes */
    case ESP_A2D_AUDIO_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "A2DP audio state: %s", s_a2d_audio_state_str[a2d->audio_stat.state]);
        s_audio_state = a2d->audio_stat.state;
        s_a2dp_playing = (a2d->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED);
        if (ESP_A2D_AUDIO_STATE_STARTED == a2d->audio_stat.state) {
            s_pkt_cnt = 0;
            /* 播放开始时再次确认 I2S/DAC 与缓存配置一致，防止 AUDIO_CFG_EVT 在连接后仍未上报。
               如果在本次连接后尚未收到 AUDIO_CFG_EVT，也尝试应用缓存配置作为后备。 */
            bt_a2dp_apply_cached_audio_cfg();
        } else {
        }
        break;
    }
    /* when audio codec is configured, this event comes */
    case ESP_A2D_AUDIO_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "A2DP audio stream configuration, codec type: %d", a2d->audio_cfg.mcc.type);
        uint32_t sample_rate = 48000;
        uint8_t bit_depth = 16;
        uint8_t channels = 2;
        codec_type_t codec_type = CODEC_UNKNOWN;

        // 从A2DP获取codec配置
        get_codec_config(a2d, &sample_rate, &bit_depth, &channels, &codec_type);

        // 转换为字符串
        const char* codec_name = codec_type_to_string(codec_type);

        ESP_LOGI(BT_AV_TAG, "A2DP audio stream configuration, codec name: %s", codec_name);

        /* LHDC：AUDIO_CFG 后主动重调 configure，保证工作区分配成功 */
#if defined(CONFIG_BT_A2DP_LHDCV5_DECODER)
        if (codec_type == CODEC_LHDCV5) {
            /* 重调入口与 AVDTP 栈内部调用一致：a2dp_lhdcv5_decoder_configure 按完整
             * AVDTP codec_info 解析（首字节 LOSC=13，随后 MT/CT 两字节，然后 CIE）。
             * 而 esp_a2d_mcc_t.cie 是去掉 3B 头的纯 CIE（codec_info+AVDT_CODEC_HEADER_SIZE，
             * 自 vendor 起 11B）——旧代码直接传 CIE，首字节=vendor 低字节≠13，
             * A2DP_ParseInfoLhdcV5 必返 A2D_WRONG_CODEC → "failed to parse LHDC V5 CIE"
             * 误报刷屏（解码器保持首次栈内配置，声音正常）。此处补回 3B 头。 */
            uint8_t codec_info[A2DP_LHDCV5_CODEC_LEN + 1];  /* 14 = LOSC + 13B(H1..P9) */
            codec_info[0] = A2DP_LHDCV5_CODEC_LEN;        /* LOSC = 13 */
            codec_info[1] = (0x00U << 4);                 /* mediaType=audio（高4位） */
            codec_info[2] = 0xFFU;                        /* codecType=NON_A2DP */
            memcpy(&codec_info[3], a2d->audio_cfg.mcc.cie.lhdcv5, ESP_A2D_CIE_LEN_LHDCV5);
            a2dp_lhdcv5_decoder_configure(codec_info);
        }
#endif
        /* 内存诊断：LHDC workspace 需 32.5KB 连续内部 SRAM（192k 满配） */
        ESP_LOGI(BT_AV_TAG, "MEM@codec cfg: internal free=%u largest=%u (need ws>=32.5KB), pheap free=%u",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

        /* 保存当前 codec 类型 */
        s_codec_type = codec_type;
        s_last_cfg_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

        /* 缓存最新配置，作为 AUDIO_CFG_EVT 丢失时的后备 */
        s_last_sample_rate = sample_rate;
        s_last_bit_depth   = bit_depth;
        s_last_channels    = channels;
        s_last_cfg_valid   = true;
        s_audio_cfg_received_after_connect = true;

        /* 将耗时的 I2S/DAC 重配置放到 BtAvBgTask 执行，避免阻塞 BtAppTask 事件队列。
           这在 LHDC V5 配置切换产生事件风暴时尤为关键。 */
        bt_av_bg_enqueue_audio_cfg(sample_rate, bit_depth, channels);
        break;
    }
    /* when a2dp init or deinit completed, this event comes */
    case ESP_A2D_PROF_STATE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if (ESP_A2D_INIT_SUCCESS == a2d->a2d_prof_stat.init_state) {
            ESP_LOGI(BT_AV_TAG, "A2DP PROF STATE: Init Complete");
        } else {
            ESP_LOGI(BT_AV_TAG, "A2DP PROF STATE: Deinit Complete");
        }
        break;
    }
    /* When protocol service capabilities configured, this event comes */
    case ESP_A2D_SNK_PSC_CFG_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "protocol service capabilities configured: 0x%x ", a2d->a2d_psc_cfg_stat.psc_mask);
        if (a2d->a2d_psc_cfg_stat.psc_mask & ESP_A2D_PSC_DELAY_RPT) {
            ESP_LOGI(BT_AV_TAG, "Peer device support delay reporting");
        } else {
            ESP_LOGI(BT_AV_TAG, "Peer device unsupport delay reporting");
        }
        break;
    }
    /* when set delay value completed, this event comes */
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        if (ESP_A2D_SET_INVALID_PARAMS == a2d->a2d_set_delay_value_stat.set_state) {
            ESP_LOGI(BT_AV_TAG, "Set delay report value: fail");
        } else {
            ESP_LOGI(BT_AV_TAG, "Set delay report value: success, delay_value: %u * 1/10 ms", a2d->a2d_set_delay_value_stat.delay_value);
        }
        break;
    }
    /* when get delay value completed, this event comes */
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
        a2d = (esp_a2d_cb_param_t *)(p_param);
        ESP_LOGI(BT_AV_TAG, "Get delay report value: delay_value: %u * 1/10 ms", a2d->a2d_get_delay_value_stat.delay_value);
        /* Default delay value plus delay caused by application layer */
        esp_a2d_sink_set_delay_value(a2d->a2d_get_delay_value_stat.delay_value + APP_DELAY_VALUE);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

/* AVRCP source 对无元数据的 attribute 返回占位文本（Unavailable / Not Provided /
 * Unknown 等），过滤后视为无数据（调用方清空字段）。
 * 参数用 void* 兼容 uint8_t*（AVRCP attr_text）与 char*（本地标题缓冲）。 */
static bool bt_av_meta_is_placeholder(const void *p)
{
    if (p == NULL) {
        return true;
    }
    const uint8_t *text = (const uint8_t *)p;
    if (text[0] == '\0') {
        return true;
    }
    /* 转小写后与常见占位文本比较 */
    char buf[20];
    int i = 0;
    for (; text[i] != '\0' && i < (int)sizeof(buf) - 1; i++) {
        char c = (char)text[i];
        if (c >= 'A' && c <= 'Z') {
            c += ('a' - 'A');
        }
        buf[i] = c;
    }
    buf[i] = '\0';
    if (strcmp(buf, "unavailable") == 0 ||
        strcmp(buf, "not provide") == 0 ||
        strcmp(buf, "not provided") == 0 ||
        strcmp(buf, "unknown") == 0 ||
        strcmp(buf, "n/a") == 0 ||
        strcmp(buf, "na") == 0) {
        return true;
    }
    return false;
}

static void bt_av_hdl_avrc_ct_evt(uint16_t event, void *p_param)
{
    int64_t t_start = esp_timer_get_time();
    (void)t_start;

    ESP_LOGD(BT_RC_CT_TAG, "%s event: %d", __func__, event);

    esp_avrc_ct_cb_param_t *rc = (esp_avrc_ct_cb_param_t *)(p_param);

    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        ESP_LOGI(BT_RC_CT_TAG, "AVRC conn_state event: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);

        if (rc->conn_stat.connected) {
            /* get remote supported event_ids of peer AVRCP Target */
            esp_avrc_ct_send_get_rn_capabilities_cmd(APP_RC_CT_TL_GET_CAPS);
        } else {
            /* clear peer notification capability record */
            s_avrc_peer_rn_cap.bits = 0;
        }
        break;
    }
    /* when passthrough responsed, this event comes */
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC passthrough rsp: key_code 0x%x, key_state %d, rsp_code %d", rc->psth_rsp.key_code,
                    rc->psth_rsp.key_state, rc->psth_rsp.rsp_code);
        break;
    }
    /* when metadata responsed, this event comes */
    case ESP_AVRC_CT_METADATA_RSP_EVT: {
        ESP_LOGD(BT_RC_CT_TAG, "AVRC metadata rsp: attribute id 0x%x, %s", rc->meta_rsp.attr_id, rc->meta_rsp.attr_text); // test OK, 值会被复制，不再单独打印刷屏
        /* 手机把歌词放在 Title(0x1)，把歌名放在 Artist(0x2)。
           因此：Artist 作为歌曲标题；Title 作为歌词。
           手机对无元数据的 attribute 返回占位文本（Unavailable / Not Provided 等），
           过滤后清空对应字段，避免把占位字符串当真实标题/歌词。 */
        bool placeholder = bt_av_meta_is_placeholder(rc->meta_rsp.attr_text);
        if (rc->meta_rsp.attr_id == ESP_AVRC_MD_ATTR_TITLE) {
            if (placeholder) {
                s_title[0] = '\0';
                ESP_LOGI(BT_RC_CT_TAG, "Lyrics: (placeholder, cleared)");
            } else {
                strncpy(s_title, (const char *)rc->meta_rsp.attr_text, sizeof(s_title) - 1);
                s_title[sizeof(s_title) - 1] = '\0';
                ESP_LOGI(BT_RC_CT_TAG, "Lyrics: %s", s_title);
            }
        } else if (rc->meta_rsp.attr_id == ESP_AVRC_MD_ATTR_ARTIST) {
            if (placeholder) {
                s_song_title[0] = '\0';
                ESP_LOGI(BT_RC_CT_TAG, "Song title: (placeholder, cleared)");
            } else {
                strncpy(s_song_title, (const char *)rc->meta_rsp.attr_text, sizeof(s_song_title) - 1);
                s_song_title[sizeof(s_song_title) - 1] = '\0';
                ESP_LOGI(BT_RC_CT_TAG, "Song title: %s", s_song_title);
            }
        free(rc->meta_rsp.attr_text);
        break;
    }
    /* when notified, this event comes */
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC event notification: %d", rc->change_ntf.event_id); // test OK, 留着方便确认蓝牙状态
        bt_av_notify_evt_handler(rc->change_ntf.event_id, &rc->change_ntf.event_parameter);
        break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "AVRC remote features %"PRIx32", TG features %x", rc->rmt_feats.feat_mask, rc->rmt_feats.tg_feat_flag);
        break;
    }
    /* when notification capability of peer device got, this event comes */
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
        ESP_LOGI(BT_RC_CT_TAG, "remote rn_cap: count %d, bitmask 0x%x", rc->get_rn_caps_rsp.cap_count,
                 rc->get_rn_caps_rsp.evt_set.bits);
        s_avrc_peer_rn_cap.bits = rc->get_rn_caps_rsp.evt_set.bits;
        /* 初次连接：清空旧标题/歌词，准备接收新数据 */
        s_song_title[0] = '\0';
        s_title[0] = '\0';
        memset(s_current_track_uid, 0, sizeof(s_current_track_uid));
        bt_av_new_track();
        bt_av_playback_changed();
        bt_av_play_pos_changed();
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_RC_CT_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
    }
}

static void bt_av_hdl_avrc_tg_evt(uint16_t event, void *p_param)
{
    ESP_LOGD(BT_RC_TG_TAG, "%s event: %d", __func__, event);

    esp_avrc_tg_cb_param_t *rc = (esp_avrc_tg_cb_param_t *)(p_param);

    switch (event) {
    /* when connection state changed, this event comes */
    case ESP_AVRC_TG_CONNECTION_STATE_EVT: {
        uint8_t *bda = rc->conn_stat.remote_bda;
        ESP_LOGI(BT_RC_TG_TAG, "AVRC conn_state evt: state %d, [%02x:%02x:%02x:%02x:%02x:%02x]",
                 rc->conn_stat.connected, bda[0], bda[1], bda[2], bda[3], bda[4], bda[5]);
        if (rc->conn_stat.connected) {
            /* AVRCP battery notification not registered (breaks absolute volume) */
            ESP_LOGD(BT_RC_TG_TAG, "AVRC TG connected");
        } else {
            ESP_LOGD(BT_RC_TG_TAG, "AVRC TG disconnected");
        }
        break;
    }
    /* when passthrough commanded, this event comes */
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC passthrough cmd: key_code 0x%x, key_state %d", rc->psth_cmd.key_code, rc->psth_cmd.key_state);
        break;
    }
    /* when absolute volume command from remote device set, this event comes */
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC set absolute volume: %d%%", (int)rc->set_abs_vol.volume * 100 / 0x7f);
        volume_set_by_controller(rc->set_abs_vol.volume);
        break;
    }
    /* when notification registered, this event comes */
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC register event notification: %d, param: 0x%"PRIx32, rc->reg_ntf.event_id, rc->reg_ntf.event_parameter);
        if (rc->reg_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            s_volume_notify = true;
            esp_avrc_rn_param_t rn_param;
            rn_param.volume = s_volume;
            esp_avrc_tg_send_rn_rsp(ESP_AVRC_RN_VOLUME_CHANGE, ESP_AVRC_RN_RSP_INTERIM, &rn_param);
        }
        break;
    }
    /* when feature of remote device indicated, this event comes */
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT: {
        ESP_LOGI(BT_RC_TG_TAG, "AVRC remote features: %"PRIx32", CT features: %x", rc->rmt_feats.feat_mask, rc->rmt_feats.ct_feat_flag);
        break;
    }
    /* others */
    default:
        ESP_LOGE(BT_RC_TG_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

/********************************
 * EXTERNAL FUNCTION DEFINITIONS
 *******************************/

void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
    case ESP_A2D_AUDIO_STATE_EVT:
    case ESP_A2D_AUDIO_CFG_EVT:
    case ESP_A2D_PROF_STATE_EVT:
    case ESP_A2D_SNK_PSC_CFG_EVT:
    case ESP_A2D_SNK_SET_DELAY_VALUE_EVT:
    case ESP_A2D_SNK_GET_DELAY_VALUE_EVT: {
        bt_app_work_dispatch(bt_av_hdl_a2d_evt, event, param, sizeof(esp_a2d_cb_param_t), NULL);
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "Invalid A2DP event: %d", event);
        break;
    }
}

void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    /* 真实数据流诊断（2026-08-18）：区分“解码器配置成功但数据没流下来”与
     * “数据在到 DAC 前被丢” */
    static uint32_t s_rx = 0;
    static uint32_t s_rx_t0 = 0;
    uint32_t now_rx = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_rx += len;
    if (now_rx - s_rx_t0 >= 2000) {
        ESP_LOGD(BT_AV_TAG, "A2DP RX: %u bytes/2s", (unsigned)s_rx); // test OK
        s_rx = 0;
        s_rx_t0 = now_rx;
    }
    // write_ringbuf(data, len);
    audio_output_i2s_write(data, len);

    /* log the number every 1000 packets in debug builds only */
    if (++s_pkt_cnt % 1000 == 0) {
        ESP_LOGD(BT_AV_TAG, "Audio packet count: %"PRIu32, s_pkt_cnt);
    }
}

void bt_app_rc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_CT_METADATA_RSP_EVT:
        bt_app_alloc_meta_buffer(param);
        /* fall through */
    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
    case ESP_AVRC_CT_PASSTHROUGH_RSP_EVT:
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
    case ESP_AVRC_CT_REMOTE_FEATURES_EVT:
    case ESP_AVRC_CT_GET_RN_CAPABILITIES_RSP_EVT: {
        bt_app_work_dispatch(bt_av_hdl_avrc_ct_evt, event, param, sizeof(esp_avrc_ct_cb_param_t), NULL);
        break;
    }
    default:
        ESP_LOGE(BT_RC_CT_TAG, "Invalid AVRC event: %d", event);
        break;
    }
}

void bt_app_rc_tg_cb(esp_avrc_tg_cb_event_t event, esp_avrc_tg_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_TG_CONNECTION_STATE_EVT:
    case ESP_AVRC_TG_REMOTE_FEATURES_EVT:
    case ESP_AVRC_TG_PASSTHROUGH_CMD_EVT:
    case ESP_AVRC_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
    case ESP_AVRC_TG_REGISTER_NOTIFICATION_EVT:
    case ESP_AVRC_TG_SET_PLAYER_APP_VALUE_EVT:
        bt_app_work_dispatch(bt_av_hdl_avrc_tg_evt, event, param, sizeof(esp_avrc_tg_cb_param_t), NULL);
        break;
    default:
        ESP_LOGE(BT_RC_TG_TAG, "Invalid AVRC event: %d", event);
        break;
    }
}

/*******************************
 * GAP / STACK EVENT HANDLERS
 ******************************/

static void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    uint8_t *bda = NULL;

    switch (event) {
    case ESP_BT_GAP_AUTH_CMPL_EVT: {
        if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(BT_AV_TAG, "authentication success: %s", param->auth_cmpl.device_name);
            esp_log_buffer_hex(BT_AV_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
        } else {
            ESP_LOGE(BT_AV_TAG, "authentication failed, status: %d", param->auth_cmpl.stat);
        }
        break;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    case ESP_BT_GAP_CFM_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %"PRIu32, param->cfm_req.num_val);
        esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
        break;
    case ESP_BT_GAP_KEY_NOTIF_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey: %"PRIu32, param->key_notif.passkey);
        break;
    case ESP_BT_GAP_KEY_REQ_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
        break;
#endif

    case ESP_BT_GAP_MODE_CHG_EVT:
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode: %d", param->mode_chg.mode);
        break;
    case ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        bda = (uint8_t *)param->acl_conn_cmpl_stat.bda;
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_ACL_CONN_CMPL_STAT_EVT Connected to [%02x:%02x:%02x:%02x:%02x:%02x], status: 0x%x",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], param->acl_conn_cmpl_stat.stat);
        /* ACL 建立后立即保持链路 ACTIVE（禁用 sniff）：A2DP/AVRCP 协商期间
         * 射频保持活跃，避免进入 sniff 拖慢 codec 协商 */
        bt_av_keep_active(bda);
        break;
    case ESP_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        bda = (uint8_t *)param->acl_disconn_cmpl_stat.bda;
        ESP_LOGI(BT_AV_TAG, "ESP_BT_GAP_ACL_DISC_CMPL_STAT_EVT Disconnected from [%02x:%02x:%02x:%02x:%02x:%02x], reason: 0x%x",
                 bda[0], bda[1], bda[2], bda[3], bda[4], bda[5], param->acl_disconn_cmpl_stat.reason);
        break;
    default:
        ESP_LOGI(BT_AV_TAG, "event: %d", event);
        break;
    }
}

void bt_av_hdl_stack_evt(uint16_t event, void *p_param)
{
    (void)p_param;
    ESP_LOGD(BT_AV_TAG, "%s event: %d", __func__, event);

    switch (event) {
    case BT_APP_EVT_STACK_UP: {
        ESP_ERROR_CHECK(esp_bt_dev_set_device_name(LOCAL_DEVICE_NAME));
        ESP_ERROR_CHECK(esp_bt_gap_register_callback(bt_app_gap_cb));

        ESP_ERROR_CHECK(esp_avrc_ct_init());
        esp_avrc_ct_register_callback(bt_app_rc_ct_cb);
        ESP_ERROR_CHECK(esp_avrc_tg_init());
        esp_avrc_tg_register_callback(bt_app_rc_tg_cb);


        esp_avrc_rn_evt_cap_mask_t evt_set = {0};
        esp_avrc_rn_evt_bit_mask_operation(ESP_AVRC_BIT_MASK_OP_SET, &evt_set, ESP_AVRC_RN_VOLUME_CHANGE);
        /* 不注册 BATTERY_STATUS_CHANGE：会破坏绝对音量的双向同步 */
        ESP_ERROR_CHECK(esp_avrc_tg_set_rn_evt_cap(&evt_set));

        ESP_ERROR_CHECK(esp_a2d_sink_init());
        esp_a2d_register_callback(&bt_app_a2d_cb);
        esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);

        /* Get the default value of the delay value */
        ESP_ERROR_CHECK(esp_a2d_sink_get_delay_value());

        /* set discoverable and connectable mode, wait to be connected */
        ESP_ERROR_CHECK(esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE));
        break;
    }
    default:
        ESP_LOGE(BT_AV_TAG, "%s unhandled event: %d", __func__, event);
        break;
    }
}

#if (CONFIG_EXAMPLE_A2DP_SINK_AUTO_RECONNECT == true)
void bt_reconnect(void)
{
    bt_av_bg_enqueue_reconnect();
}
#endif

