/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "freertos/xtensa_api.h"
#include "freertos/FreeRTOSConfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bt_app_core.h"
#include "task_config.h"

/*******************************
 * STATIC FUNCTION DECLARATIONS
 ******************************/

/* handler for application task */
static void bt_app_task_handler(void *arg);
/* message sender */
static bool bt_app_send_msg(bt_app_msg_t *msg);
/* handle dispatched messages */
static void bt_app_work_dispatched(bt_app_msg_t *msg);

/*******************************
 * STATIC VARIABLE DEFINITIONS
 ******************************/

static QueueHandle_t s_bt_app_task_queue = NULL;  /* handle of work queue */
static TaskHandle_t s_bt_app_task_handle = NULL;  /* handle of application task  */

/*******************************
 * STATIC FUNCTION DEFINITIONS
 ******************************/

static bool bt_app_send_msg(bt_app_msg_t *msg)
{
    if (msg == NULL) {
        return false;
    }

    /* 防御：bt_player_stop 会删除应用任务队列，若协议栈停止过程中仍有回调
     * （AVRCP/A2DP 断开事件）尝试投递，直接返回避免 xQueueSend 空指针崩溃 */
    if (s_bt_app_task_queue == NULL) {
        ESP_LOGW(BT_APP_CORE_TAG, "%s queue not ready, drop msg", __func__);
        return false;
    }

    /* send the message to work queue, increase timeout to avoid failure during codec switch */
    if (xQueueSend(s_bt_app_task_queue, msg, 1000 / portTICK_PERIOD_MS) != pdTRUE) {
        ESP_LOGE(BT_APP_CORE_TAG, "%s xQueue send failed", __func__);
        return false;
    }
    return true;
}

static void bt_app_work_dispatched(bt_app_msg_t *msg)
{
    if (msg->cb) {
        msg->cb(msg->event, msg->param);
    }
}

static void bt_app_task_handler(void *arg)
{
    bt_app_msg_t msg;

    for (;;) {
        /* receive message from work queue and handle it */
        if (pdTRUE == xQueueReceive(s_bt_app_task_queue, &msg, (TickType_t)portMAX_DELAY)) {
            ESP_LOGD(BT_APP_CORE_TAG, "%s, signal: 0x%x, event: 0x%x", __func__, msg.sig, msg.event);

            switch (msg.sig) {
            case BT_APP_SIG_WORK_DISPATCH:
                bt_app_work_dispatched(&msg);
                break;
            default:
                ESP_LOGW(BT_APP_CORE_TAG, "%s, unhandled signal: %d", __func__, msg.sig);
                break;
            } /* switch (msg.sig) */

            if (msg.param) {
                free(msg.param);
            }
        }
    }
}

/********************************
 * EXTERNAL FUNCTION DEFINITIONS
 *******************************/

bool bt_app_work_dispatch(bt_app_cb_t p_cback, uint16_t event, void *p_params, int param_len, bt_app_copy_cb_t p_copy_cback)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s event: 0x%x, param len: %d", __func__, event, param_len);

    bt_app_msg_t msg;
    memset(&msg, 0, sizeof(bt_app_msg_t));

    msg.sig = BT_APP_SIG_WORK_DISPATCH;
    msg.event = event;
    msg.cb = p_cback;

    if (param_len == 0) {
        return bt_app_send_msg(&msg);
    } else if (p_params && param_len > 0) {
        if ((msg.param = malloc(param_len)) != NULL) {
            memcpy(msg.param, p_params, param_len);
            /* check if caller has provided a copy callback to do the deep copy */
            if (p_copy_cback) {
                p_copy_cback(msg.param, p_params, param_len);
            }
            return bt_app_send_msg(&msg);
        }
    }

    return false;
}

void bt_app_task_start_up(void)
{
    /* 队列深度加大到 100，避免切换 LHDC V5 配置时的事件风暴导致关键事件丢失 */
    s_bt_app_task_queue = xQueueCreate(100, sizeof(bt_app_msg_t));
    /* 任务优先级使用 configMAX_PRIORITIES - 10，与 ESP32-A2DP 库默认一致。
       固定到 Core 1，与 ESP32-A2DP 保持一致，避免与 Bluedroid BTC 任务（通常
       在 Core 0）争抢 CPU，确保事件队列能被及时消费。 */
    xTaskCreatePinnedToCore(bt_app_task_handler, "BtAppTask", BT_APP_STACK_SIZE, NULL,
                            BT_APP_PRIORITY, &s_bt_app_task_handle, BT_APP_CORE);
}

void bt_app_task_shut_down(void)
{
    if (s_bt_app_task_handle) {
        vTaskDelete(s_bt_app_task_handle);
        s_bt_app_task_handle = NULL;
    }
    if (s_bt_app_task_queue) {
        vQueueDelete(s_bt_app_task_queue);
        s_bt_app_task_queue = NULL;
    }
}

/* 以下 I2S/ringbuffer API 已由 audio_output_i2s 组件接管，当前项目不再使用。
   保留空实现以维持与原始例程的符号兼容，防止意外调用时产生链接错误。 */
void bt_i2s_task_start_up(void)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s: not used, I2S is managed by audio_output_i2s", __func__);
}

void bt_i2s_task_shut_down(void)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s: not used, I2S is managed by audio_output_i2s", __func__);
}

size_t write_ringbuf(const uint8_t *data, size_t size)
{
    ESP_LOGD(BT_APP_CORE_TAG, "%s: not used, ringbuffer is managed by audio_output_i2s", __func__);
    return size;
}
