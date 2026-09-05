#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_timer.h"

#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "driver/i2s_tdm.h"
#include "soc/soc_caps.h"

#include "audio_output_i2s.h"
#include "task_config.h"

// 模拟开关控制宏

static const char *TAG = "AUDIO_OUTPUT_I2S";

typedef struct {
    i2s_chan_handle_t tx_chan;
    RingbufHandle_t ringbuf;
    SemaphoreHandle_t write_semaphore;
    bool is_initialized;
    bool is_started;
    uint32_t sample_rate;
    uint8_t bit_depth;
    uint8_t channels;
    uint8_t system_volume;  /* 系统音量 0-127（AVRCP 同级） */
    uint8_t dac_volume;     /* 实际写入 DAC 的音量 0-100（经映射后） */
    float gain;             /* 软件增益（PCM5102A 无硬件音量） */
    i2s_role_t mcu_i2s_mode;
    mcu_i2s_fmt_t mcu_i2s_format;
} audio_output_i2s_t;

static audio_output_i2s_t s_i2s_out = {0};

/* DMA 冲刷静音缓冲：16bit 播放时 DMA = 4 descs × 240 帧 × 4B = 3840 字节。
 * 原 240*4（1920B=480 帧）只覆盖一半 DMA——start 后先播静音 10.9ms 再播
 * 旧歌残留 10.9ms（实测 "stop flush: wrote 1920/1920" 仅半量）→ 切歌残留源。
 * 240*8 = 3840 字节完整覆盖 960 帧。 */
static int16_t s_dma_flush_silence[240 * 8];

/* 互斥锁：保护 I2S channel 生命周期（configure 的 del/re-create、stop、start）
 * 与消费任务对 tx_chan 的写入。蓝牙/本地切换时 configure 会删除并重建 channel，
 * 若消费任务或蓝牙 BtAvBgTask 同时访问旧 channel 指针会 use-after-free 崩溃。 */
static SemaphoreHandle_t s_i2s_mutex = NULL;

// 原配置32KB在iOS高码率下容易溢出，增大到64KB提供更充足的缓冲
// 本地 Vorbis 解码速度接近实时，再增大到96KB以吸收解码抖动，避免频繁 underflow
// 2026-08-08：蓝牙 LDAC 96kHz（768KB/s）双模共存下偶发 underflow，再增大到
// 160KB（≈208ms 缓冲）吸收射频/共存抖动，避免短暂静音卡顿
#define RINGBUF_SIZE (160 * 1024)                // 内部 RAM 版（PSRAM ringbuffer 双核并发缓存一致性风险）
#define RINGBUF_PREFETCH_WATER_LEVEL (80 * 1024) // 预取阈值 48KB → 80KB
#define RINGBUF_MAX_WATER_LEVEL (160 * 1024)     // 与RINGBUF_SIZE保持一致

typedef enum {
    RINGBUFFER_MODE_PROCESSING,
    RINGBUFFER_MODE_PREFETCHING,
    RINGBUFFER_MODE_DROPPING
} audio_sink_ringbuffer_mode_t;

static audio_sink_ringbuffer_mode_t s_ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;

/* 连续无数据毫秒数（i2s_task 内更新，播放异常检测读取） */
volatile uint32_t s_i2s_starved_ms = 0;
static volatile uint32_t s_last_real_write_ms = 0;  /* 最近一次 i2s_write 真实成功的 tick/ms; 用于判 "is_started 但上游长期无数据" 状态 (如 A2DP 已连未播) - 抑制 STARVED 持续刷屏 */
static volatile uint32_t s_starve_episode_cnt = 0;   /* ≥500ms 饥饿事件累计 */
uint32_t audio_output_i2s_get_starve_episodes(void)
{
    return s_starve_episode_cnt;
}
static uint32_t s_dma_bytes = 0;
static uint32_t s_dma_t0 = 0;

/* 软件音量：PCM5102A 无硬件音量，按增益缩放 32-bit PCM */
static inline void audio_output_apply_gain(int32_t *pcm, size_t n, float g)
{
    for (size_t i = 0; i < n; i++) {
        pcm[i] = (int32_t)(pcm[i] * g);
    }
}

static void audio_output_i2s_task_handler(void *arg)
{
    /* 诊断：i2s_task 栈水位（黑屏调查——栈溢出踩 heap 会损坏 PSRAM 缓冲） */
    ESP_LOGI(TAG, "i2s_task HWM=%u/8192",
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
    uint8_t *data = NULL;
    size_t item_size = 0;
    // 增大每次读取的数据量，提高I2S输出效率
    // 原240*6=1440字节，改为240*12=2880字节，减少读取次数
    const size_t item_size_upto = 240 * 12;
    size_t bytes_written = 0;
    static uint32_t s_starve_episode_ms = 0;   /* 连续饥饿段计时（诚实告警用） */

    for (;;) {
        if (pdTRUE == xSemaphoreTake(s_i2s_out.write_semaphore, portMAX_DELAY)) {
            for (;;) {
                item_size = 0;
                data = (uint8_t *)xRingbufferReceiveUpTo(s_i2s_out.ringbuf, &item_size,
                                                        (TickType_t)pdMS_TO_TICKS(20), item_size_upto);
                if (item_size == 0) {
                    /* 持续消费（2026-08-16 二次实施）：ringbuffer 空时空转轮询而非
                     * break 回外层等信号量——脉冲消费（攒 80KB 才恢复）期间 DMA 缓冲
                     * 耗尽导致可闻断音。生产侧已优化（memmove 水印压缩 + SD 预读线程，
                     * 生产 2 倍速），持续消费下 ringbuffer 保持高水位、DMA 不空转。
                     * 连续无数据 ≥500ms（暂停/停止）才归零频谱。
                     * I2S 未启动（暂停/停止）时不计饥饿——用户反馈暂停后 STARVED 刷屏。 */
                    if (!s_i2s_out.is_started) {
                        vTaskDelay(pdMS_TO_TICKS(20));
                        continue;
                    }
                    /* 已 start 但上游长期没数据 (如 A2DP 已连未播 / DLNA 已连未播):
                     * 5s 内仍算 "短暂饥饿" 继续告警; 超过 5s 视为 "上游停播"
                     * 停止累加, 避免持续刷屏. 收到任何一帧数据即复位. */
                    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    /* 从未真实写入过 (s_last_real_write_ms=0) 或距上次写入 > 5s
                     * 都视为 "上游停播" 立即静默, 避免刷屏.
                     * 5s 内才允许告警 (短暂卡顿的真实证据) */
                    if ((int32_t)(now_ms - s_last_real_write_ms) > 5000) {
                        s_i2s_starved_ms = 0;
                        s_starve_episode_ms = 0;
                        vTaskDelay(pdMS_TO_TICKS(20));
                        continue;
                    }
                    s_i2s_starved_ms += 20;
                    s_starve_episode_ms += 20;
                    /* 诚实饥饿告警（2026-08-22）：DLNA 排查发现"无 starved 日志"不可
                     * 作为流畅证据——本路径此前根本没有饥饿打印。≥200ms 即报，之后
                     * 每 1s 追一条；收到任何数据即结束一段。 */
                    if (s_starve_episode_ms == 200) {
                        ESP_LOGW(TAG, "I2S STARVED %ums (episode start)", s_starve_episode_ms);
                    } else if (s_starve_episode_ms > 200 &&
                               s_starve_episode_ms % 1000 == 0) {
                        ESP_LOGW(TAG, "I2S STARVED %ums ...", s_starve_episode_ms);
                    }
                    if (s_i2s_starved_ms >= 500) {
                        s_i2s_starved_ms = 0;
                        s_starve_episode_cnt++;   /* DLNA 轮换触发依据：饥饿事件累计 */
                    }
                    continue;
                }
                s_starve_episode_ms = 0;
                s_i2s_starved_ms = 0;

                /* 消费节拍诊断：每 1000 次循环打印消费速率（对比播放速率判断
                 * i2s_task 是否被 DMA 正确节流） */
                {
                    static uint32_t s_loop_cnt = 0;
                    static int64_t s_loop_t0 = esp_timer_get_time();
                    if (++s_loop_cnt >= 1000) {
                        int64_t dt = esp_timer_get_time() - s_loop_t0;
                        ESP_LOGD(TAG, "i2s_task: 1000 loops in %lldms (%.2f us/loop)",
                                 (long long)(dt / 1000), (double)dt / 1000.0);
                        s_loop_cnt = 0;
                        s_loop_t0 = esp_timer_get_time();
                    }
                }

                /* 已停止（audio_output_i2s_stop 置 is_started=false）：
                 * 丢弃已取出的数据、归零频谱、不再写 I2S。
                 * 否则 stop 后任务仍可能把旧 PCM 写入 DAC（挂载后播放出现残留音频），
                 * 且 i2s_channel_write(portMAX_DELAY) 在 disable 后可能阻塞导致任务
                 * 不再执行 underflow 归零（熄屏频谱残留）。 */
                if (!s_i2s_out.is_started) {
                    vRingbufferReturnItem(s_i2s_out.ringbuf, data);
                    break;
                }

                /* 持锁访问 tx_chan：与 configure 的 del/re-create、stop/start 互斥。
                 * 蓝牙→本地切换时 configure 会 i2s_del_channel + 重建，若此处不持锁，
                 * 可能在写 I2S 时 channel 已被删除 → use-after-free 崩溃。 */
                if (s_i2s_mutex != NULL) {
                    xSemaphoreTake(s_i2s_mutex, portMAX_DELAY);
                }

                /* 软件音量（PCM5102A 无硬件音量） */
                if (s_i2s_out.gain < 0.999f) {
                    audio_output_apply_gain((int32_t *)data, item_size / 4, s_i2s_out.gain);
                }
                /* 写 I2S 用超时而非永久阻塞：I2S 被 stop disable 后 write 会失败/阻塞 */
                i2s_channel_write(s_i2s_out.tx_chan, data, item_size, &bytes_written, pdMS_TO_TICKS(50));
                if (bytes_written != item_size) {
                    static uint32_t s_drop_cnt = 0;
                    if (++s_drop_cnt <= 10 || (s_drop_cnt % 100) == 0) {
                        ESP_LOGW(TAG, "I2S write short: wrote=%u/%u (DMA busy)",
                                 (unsigned)bytes_written, (unsigned)item_size);
                    }
                }

                if (s_i2s_mutex != NULL) {
                    xSemaphoreGive(s_i2s_mutex);
                }
                s_dma_bytes += (uint32_t)bytes_written;
                {
                    uint32_t now_dma = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    if (now_dma - s_dma_t0 >= 2000) {
                        ESP_LOGD(TAG, "DMA WRITE: %u bytes/2s (rb starve %ums)",
                                 (unsigned)s_dma_bytes, (unsigned)s_i2s_starved_ms);    // test OK
                        s_dma_bytes = 0;
                        s_dma_t0 = now_dma;
                    }
                }
                vRingbufferReturnItem(s_i2s_out.ringbuf, (void *)data);
            }
        }
    }
}

esp_err_t audio_output_i2s_init(uint32_t sample_rate, uint8_t bit_depth, uint8_t channels, 
                                mcu_i2s_fmt_t mcu_i2s_format, i2s_role_t mcu_i2s_mode)
{
    if (s_i2s_out.is_initialized) {
        ESP_LOGW(TAG, "I2S output already initialized");
        return ESP_OK;
    }

    /* 创建互斥锁（configure 删除/重建 channel 时与消费任务/BtAvBgTask 互斥） */
    if (s_i2s_mutex == NULL) {
        s_i2s_mutex = xSemaphoreCreateMutex();
        if (s_i2s_mutex == NULL) {
            ESP_LOGE(TAG, "Failed to create I2S mutex");
            return ESP_FAIL;
        }
    }

    char mcu_i2s_format_str[26] = "";

    gpio_num_t I2S_BCK = (gpio_num_t)CONFIG_I2S_BCK_PIN;
    gpio_num_t I2S_LRCK = (gpio_num_t)CONFIG_I2S_LRCK_PIN;
    gpio_num_t I2S_DATA = (gpio_num_t)CONFIG_I2S_DATA_PIN;

    // 硬件重置
    gpio_reset_pin(I2S_BCK);
    gpio_reset_pin(I2S_LRCK);
    gpio_reset_pin(I2S_DATA);
    // 设置输出模式
    gpio_set_direction(I2S_BCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(I2S_LRCK, GPIO_MODE_OUTPUT);
    gpio_set_direction(I2S_DATA, GPIO_MODE_OUTPUT);
    // 提升驱动能力（最大）
    gpio_set_drive_capability(I2S_BCK, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(I2S_LRCK, GPIO_DRIVE_CAP_3);
    gpio_set_drive_capability(I2S_DATA, GPIO_DRIVE_CAP_3);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 假如已经设置了参数，则不要在此函数内清零s_i2s_out
    // memset(&s_i2s_out, 0, sizeof(audio_output_i2s_t));

    i2s_data_bit_width_t bit_width;
    switch (bit_depth) {
        case 16:
            bit_width = I2S_DATA_BIT_WIDTH_16BIT;
            break;
        case 24:
            bit_width = I2S_DATA_BIT_WIDTH_24BIT;
            break;
        case 32:
            bit_width = I2S_DATA_BIT_WIDTH_32BIT;
            break;
        default:
            ESP_LOGE(TAG, "Unsupported bit depth: %d", bit_depth);
            return ESP_ERR_INVALID_ARG;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, (i2s_role_t)mcu_i2s_mode);
    chan_cfg.auto_clear = true;
    chan_cfg.dma_desc_num = 4;  // Reduced from 6 to save memory
    /* 保持 DMA 缓冲时间恒定：位深越小每帧字节数越少，需增大 frame_num。
     * 32bit 时 120 帧约 21.8ms；16bit 时 240 帧同样约 21.8ms，避免 16bit 下缓冲过短导致 DMA 欠载噼啪。 */
    chan_cfg.dma_frame_num = (120 * 32) / bit_depth;

    i2s_slot_mode_t slot_channels = (channels == 1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_i2s_out.tx_chan, NULL));

    switch (mcu_i2s_format) {
        case I2S_STD_FORMAT: {
                i2s_std_config_t std_cfg = {
                    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
                    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bit_width, slot_channels),
                    .gpio_cfg = {
                        .mclk = I2S_GPIO_UNUSED,
                        .bclk = I2S_BCK,
                        .ws = I2S_LRCK,
                        .dout = I2S_DATA,
                        .din = I2S_GPIO_UNUSED,
                        .invert_flags = {
                            .mclk_inv = false,
                            .bclk_inv = false,
                            .ws_inv = false,
                        },
                    },
                };
                ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_out.tx_chan, &std_cfg));
            }
            strcpy(mcu_i2s_format_str, "I2S_STD_FORMAT");
            break;
        case I2S_LSB_FORMAT:
            ESP_LOGE(TAG, "Unsupported I2S format: %d", mcu_i2s_format);
            strcpy(mcu_i2s_format_str, "UNSUPPORTED");
            return ESP_FAIL;
        case I2S_MSB_FORMAT: {
                i2s_std_config_t std_cfg = {
                    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
                    .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(bit_width, slot_channels),
                    .gpio_cfg = {
                        .mclk = I2S_GPIO_UNUSED,
                        .bclk = I2S_BCK,
                        .ws = I2S_LRCK,
                        .dout = I2S_DATA,
                        .din = I2S_GPIO_UNUSED,
                        .invert_flags = {
                            .mclk_inv = false,
                            .bclk_inv = false,
                            .ws_inv = false,
                        },
                    },
                };
                ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_out.tx_chan, &std_cfg));
            }
            strcpy(mcu_i2s_format_str, "I2S_MSB_FORMAT");
            break;
        case I2S_PHILIPS_FORMAT: {
                i2s_std_config_t std_cfg = {
                    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
                    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bit_width, slot_channels),
                    .gpio_cfg = {
                        .mclk = I2S_GPIO_UNUSED,
                        .bclk = I2S_BCK,
                        .ws = I2S_LRCK,
                        .dout = I2S_DATA,
                        .din = I2S_GPIO_UNUSED,
                        .invert_flags = {
                            .mclk_inv = false,
                            .bclk_inv = false,
                            .ws_inv = false,
                        },
                    },
                };
                ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_out.tx_chan, &std_cfg));
            }
            strcpy(mcu_i2s_format_str, "I2S_PHILIPS_FORMAT");
            break;
        case I2S_RIGHT_JUSTIFIED_FORMAT:
            ESP_LOGE(TAG, "Unsupported I2S format: %d", mcu_i2s_format);
            strcpy(mcu_i2s_format_str, "UNSUPPORTED");
            return ESP_FAIL;
        case I2S_LEFT_JUSTIFIED_FORMAT: {
                i2s_std_config_t std_cfg = {
                    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
                    .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(bit_width, slot_channels),
                    .gpio_cfg = {
                        .mclk = I2S_GPIO_UNUSED,
                        .bclk = I2S_BCK,
                        .ws = I2S_LRCK,
                        .dout = I2S_DATA,
                        .din = I2S_GPIO_UNUSED,
                        .invert_flags = {
                            .mclk_inv = false,
                            .bclk_inv = false,
                            .ws_inv = false,
                        },
                    },
                };
                ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_out.tx_chan, &std_cfg));
            }
            strcpy(mcu_i2s_format_str, "I2S_JUSTIFIED_FORMAT");
            break;
        case I2S_PCM: {
                i2s_std_config_t std_cfg = {
                    .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
                    .slot_cfg = I2S_STD_PCM_SLOT_DEFAULT_CONFIG(bit_width, slot_channels),
                    .gpio_cfg = {
                        .mclk = I2S_GPIO_UNUSED,
                        .bclk = I2S_BCK,
                        .ws = I2S_LRCK,
                        .dout = I2S_DATA,
                        .din = I2S_GPIO_UNUSED,
                        .invert_flags = {
                            .mclk_inv = false,
                            .bclk_inv = false,
                            .ws_inv = false,
                        },
                    },
                };
                ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_out.tx_chan, &std_cfg));
            }
            strcpy(mcu_i2s_format_str, "I2S_PCM");
            break;
        default:
            ESP_LOGE(TAG, "Unsupported I2S format: %d", mcu_i2s_format);
            strcpy(mcu_i2s_format_str, "UNSUPPORTED");
            return ESP_ERR_INVALID_ARG;
    }

    // update params
    s_i2s_out.sample_rate = sample_rate;
    s_i2s_out.bit_depth = bit_depth;
    s_i2s_out.channels = channels;
    s_i2s_out.mcu_i2s_mode = mcu_i2s_mode;

    s_i2s_out.ringbuf = xRingbufferCreate(RINGBUF_SIZE, RINGBUF_TYPE_BYTEBUF);
    if (s_i2s_out.ringbuf == NULL) {
        ESP_LOGE(TAG, "Failed to create ringbuffer");
        return ESP_FAIL;
    }

    s_i2s_out.write_semaphore = xSemaphoreCreateBinary();
    if (s_i2s_out.write_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create semaphore");
        vRingbufferDelete(s_i2s_out.ringbuf);
        return ESP_FAIL;
    }

    /* Pin I2S render task to Core 0, away from Bluedroid A2DP/decoder task which
       runs on Core 1. This keeps the audio output from competing with LHDC decode.
       Stack 8192：频谱 FFT 局部数组 ~2.5KB + DSP 5 段链——4096 曾接近上限，
       溢出会踩坏相邻 heap（PSRAM 缓冲/s_fb）→ 偶发黑屏/冻结（2026-08-16 调查） */
    xTaskCreatePinnedToCore(audio_output_i2s_task_handler, "i2s_task", I2S_TASK_STACK_SIZE, NULL, I2S_TASK_PRIORITY, NULL, I2S_TASK_CORE);

    s_i2s_out.is_initialized = true;
    s_i2s_out.is_started = false;

    ESP_LOGI(TAG, "I2S output initialized: %dHz | %dbit | %dch | %s | %s | dma=%d descs * %d frames",
             s_i2s_out.sample_rate, s_i2s_out.bit_depth, s_i2s_out.channels, mcu_i2s_format_str,
             (s_i2s_out.mcu_i2s_mode == I2S_ROLE_MASTER ? "I2S_ROLE_MASTER" : "I2S_ROLE_SLAVE"),
             chan_cfg.dma_desc_num, chan_cfg.dma_frame_num);

    return ESP_OK;
}

void audio_output_i2s_cleanup(void)
{
    if (!s_i2s_out.is_initialized) {
        return;
    }

    audio_output_i2s_stop();

    if (s_i2s_out.tx_chan != NULL) {
        i2s_del_channel(s_i2s_out.tx_chan);
        s_i2s_out.tx_chan = NULL;
        vTaskDelay(pdMS_TO_TICKS(50)); // wait for I2S memory to be released
    }

    if (s_i2s_out.ringbuf != NULL) {
        vRingbufferDelete(s_i2s_out.ringbuf);
        s_i2s_out.ringbuf = NULL;
    }

    if (s_i2s_out.write_semaphore != NULL) {
        vSemaphoreDelete(s_i2s_out.write_semaphore);
        s_i2s_out.write_semaphore = NULL;
    }

    s_i2s_out.is_initialized = false;

    ESP_LOGI(TAG, "I2S output cleaned up");
}

/* 内部加锁实现（前向声明）：实际配置逻辑在互斥锁保护下执行 */
static esp_err_t audio_output_i2s_configure_locked(uint32_t sample_rate, uint8_t bit_depth, uint8_t channels, mcu_i2s_fmt_t mcu_i2s_format);

esp_err_t audio_output_i2s_configure(uint32_t sample_rate, uint8_t bit_depth, uint8_t channels, mcu_i2s_fmt_t mcu_i2s_format)
{
    if (s_i2s_mutex != NULL) {
        xSemaphoreTake(s_i2s_mutex, portMAX_DELAY);
    }
    esp_err_t ret = audio_output_i2s_configure_locked(sample_rate, bit_depth, channels, mcu_i2s_format);
    if (s_i2s_mutex != NULL) {
        xSemaphoreGive(s_i2s_mutex);
    }
    return ret;
}

/* I2S 配置（含 bit_depth 变化时的 del/re-create channel）必须在互斥锁内执行：
 * 蓝牙→本地切换时此函数删除并重建 channel，若与消费任务写 I2S 或蓝牙 BtAvBgTask
 * 并发，会 use-after-free 崩溃。 */
static esp_err_t audio_output_i2s_configure_locked(uint32_t sample_rate, uint8_t bit_depth, uint8_t channels, mcu_i2s_fmt_t mcu_i2s_format)
{
    if (!s_i2s_out.is_initialized) {
        ESP_LOGE(TAG, "I2S output not initialized");
        return ESP_FAIL;
    }

    if (s_i2s_out.sample_rate == sample_rate &&
        s_i2s_out.bit_depth == bit_depth &&
        s_i2s_out.channels == channels) {
        ESP_LOGD(TAG, "I2S configuration unchanged");
        return ESP_OK;
    }

    char mcu_i2s_format_str[26] = "";
    char s_mcu_i2s_format_str[26] = "";

    switch (mcu_i2s_format) {
        case I2S_STD_FORMAT:
            strcpy(mcu_i2s_format_str, "I2S_STD_FORMAT");
            break;
        case I2S_LSB_FORMAT:
            strcpy(mcu_i2s_format_str, "UNSUPPORTED");
            break;
        case I2S_MSB_FORMAT:
            strcpy(mcu_i2s_format_str, "I2S_MSB_FORMAT");
            break;
        case I2S_PHILIPS_FORMAT:
            strcpy(mcu_i2s_format_str, "I2S_PHILIPS_FORMAT");
            break;
        case I2S_RIGHT_JUSTIFIED_FORMAT:
            strcpy(mcu_i2s_format_str, "UNSUPPORTED");
            break;
        case I2S_LEFT_JUSTIFIED_FORMAT:
            strcpy(mcu_i2s_format_str, "I2S_LEFT_JUSTIFIED_FORMAT");
            break;
        case I2S_PCM:
            strcpy(mcu_i2s_format_str, "I2S_PCM");
            break;
        default:
            strcpy(mcu_i2s_format_str, "UNSUPPORTED");
            break;
    }

    ESP_LOGI(TAG, "Configure I2S: %dHz | %dch | %dbit | %s", sample_rate, channels, bit_depth, mcu_i2s_format_str);

    // If bit depth changes, we need to re-create the I2S channel
    // because DMA buffer size is determined at channel creation time
    if (s_i2s_out.bit_depth != bit_depth) {
        ESP_LOGI(TAG, "Bit depth changed from %d to %d, re-creating I2S channel", 
                 s_i2s_out.bit_depth, bit_depth);
        
        // Stop and disable current channel
        if (s_i2s_out.is_started) {
            i2s_channel_disable(s_i2s_out.tx_chan);
            s_i2s_out.is_started = false;
        }
        
        // Delete current channel
        if (s_i2s_out.tx_chan != NULL) {
            i2s_del_channel(s_i2s_out.tx_chan);
            s_i2s_out.tx_chan = NULL;
            vTaskDelay(pdMS_TO_TICKS(5)); // short wait for I2S memory to be released
        }
        
        // Re-create channel with new bit depth
        i2s_data_bit_width_t bit_width;
        switch (bit_depth) {
            case 16:
                bit_width = I2S_DATA_BIT_WIDTH_16BIT;
                break;
            case 24:
                bit_width = I2S_DATA_BIT_WIDTH_24BIT;
                break;
            case 32:
                bit_width = I2S_DATA_BIT_WIDTH_32BIT;
                break;
            default:
                ESP_LOGE(TAG, "Unsupported bit depth: %d", bit_depth);
                return ESP_ERR_INVALID_ARG;
        }
        
        i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, s_i2s_out.mcu_i2s_mode);
        chan_cfg.auto_clear = true;
        chan_cfg.dma_desc_num = 4;  // Reduced from 6 to save memory
        /* 保持 DMA 缓冲时间恒定：位深越小每帧字节数越少，需增大 frame_num。
         * 32bit 时 120 帧约 21.8ms；16bit 时 240 帧同样约 21.8ms，避免 16bit 下缓冲过短导致 DMA 欠载噼啪。 */
        chan_cfg.dma_frame_num = (120 * 32) / bit_depth;
        
        ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &s_i2s_out.tx_chan, NULL));
        
        // Re-initialize with new configuration
        i2s_slot_mode_t slot_channels = (channels == 1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO;
        
        // For 24-bit data, mclk_multiple must be a multiple of 3
        i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);
        if (bit_depth == 24) {
            clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_384;  // 384 is a multiple of 3
        }
        
        switch (mcu_i2s_format) {
            case I2S_STD_FORMAT: {
                    i2s_std_config_t std_cfg = {
                        .clk_cfg = clk_cfg,
                        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bit_width, slot_channels),
                        .gpio_cfg = {
                            .mclk = I2S_GPIO_UNUSED,
                            .bclk = (gpio_num_t)CONFIG_I2S_BCK_PIN,
                            .ws = (gpio_num_t)CONFIG_I2S_LRCK_PIN,
                            .dout = (gpio_num_t)CONFIG_I2S_DATA_PIN,
                            .din = I2S_GPIO_UNUSED,
                            .invert_flags = {
                                .mclk_inv = false,
                                .bclk_inv = false,
                                .ws_inv = false,
                            },
                        },
                    };
                    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_out.tx_chan, &std_cfg));
                }
                break;
            case I2S_MSB_FORMAT: {
                    i2s_std_config_t std_cfg = {
                        .clk_cfg = clk_cfg,
                        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(bit_width, slot_channels),
                        .gpio_cfg = {
                            .mclk = I2S_GPIO_UNUSED,
                            .bclk = (gpio_num_t)CONFIG_I2S_BCK_PIN,
                            .ws = (gpio_num_t)CONFIG_I2S_LRCK_PIN,
                            .dout = (gpio_num_t)CONFIG_I2S_DATA_PIN,
                            .din = I2S_GPIO_UNUSED,
                            .invert_flags = {
                                .mclk_inv = false,
                                .bclk_inv = false,
                                .ws_inv = false,
                            },
                        },
                    };
                    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_out.tx_chan, &std_cfg));
                }
                break;
            case I2S_PHILIPS_FORMAT: {
                    i2s_std_config_t std_cfg = {
                        .clk_cfg = clk_cfg,
                        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bit_width, slot_channels),
                        .gpio_cfg = {
                            .mclk = I2S_GPIO_UNUSED,
                            .bclk = (gpio_num_t)CONFIG_I2S_BCK_PIN,
                            .ws = (gpio_num_t)CONFIG_I2S_LRCK_PIN,
                            .dout = (gpio_num_t)CONFIG_I2S_DATA_PIN,
                            .din = I2S_GPIO_UNUSED,
                            .invert_flags = {
                                .mclk_inv = false,
                                .bclk_inv = false,
                                .ws_inv = false,
                            },
                        },
                    };
                    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_out.tx_chan, &std_cfg));
                }
                break;
            case I2S_LEFT_JUSTIFIED_FORMAT: {
                    i2s_std_config_t std_cfg = {
                        .clk_cfg = clk_cfg,
                        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(bit_width, slot_channels),
                        .gpio_cfg = {
                            .mclk = I2S_GPIO_UNUSED,
                            .bclk = (gpio_num_t)CONFIG_I2S_BCK_PIN,
                            .ws = (gpio_num_t)CONFIG_I2S_LRCK_PIN,
                            .dout = (gpio_num_t)CONFIG_I2S_DATA_PIN,
                            .din = I2S_GPIO_UNUSED,
                            .invert_flags = {
                                .mclk_inv = false,
                                .bclk_inv = false,
                                .ws_inv = false,
                            },
                        },
                    };
                    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_out.tx_chan, &std_cfg));
                }
                break;
            case I2S_PCM: {
                    i2s_std_config_t std_cfg = {
                        .clk_cfg = clk_cfg,
                        .slot_cfg = I2S_STD_PCM_SLOT_DEFAULT_CONFIG(bit_width, slot_channels),
                        .gpio_cfg = {
                            .mclk = I2S_GPIO_UNUSED,
                            .bclk = (gpio_num_t)CONFIG_I2S_BCK_PIN,
                            .ws = (gpio_num_t)CONFIG_I2S_LRCK_PIN,
                            .dout = (gpio_num_t)CONFIG_I2S_DATA_PIN,
                            .din = I2S_GPIO_UNUSED,
                            .invert_flags = {
                                .mclk_inv = false,
                                .bclk_inv = false,
                                .ws_inv = false,
                            },
                        },
                    };
                    ESP_ERROR_CHECK(i2s_channel_init_std_mode(s_i2s_out.tx_chan, &std_cfg));
                }
                break;
            default:
                ESP_LOGE(TAG, "Unsupported I2S format: %d", mcu_i2s_format);
                return ESP_ERR_INVALID_ARG;
        }
        
        /* 切换位深时必须清空 ringbuffer，防止旧格式（如 32bit）残留数据
         * 与新格式（如 16bit）混合导致不连续噪声。 */
        size_t rb_item_size;
        char *rb_item;
        while ((rb_item = (char *)xRingbufferReceive(s_i2s_out.ringbuf, &rb_item_size, 0)) != NULL) {
            vRingbufferReturnItem(s_i2s_out.ringbuf, rb_item);
        }

        // update params
        s_i2s_out.sample_rate = sample_rate;
        s_i2s_out.bit_depth = bit_depth;
        s_i2s_out.channels = channels;
        s_i2s_out.mcu_i2s_format = mcu_i2s_format;
        
        // Reset ringbuffer mode to PREFETCHING for new stream
        s_ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;
        
        ESP_LOGI(TAG, "I2S channel re-created: %dHz | %dbit | %dch | dma=%d descs * %d frames (not enabled, waiting for start)",
                 sample_rate, bit_depth, channels, chan_cfg.dma_desc_num, chan_cfg.dma_frame_num);
        return ESP_OK;
    }

    /* 采样率/声道变化（位深不变）时无需重建 channel，reconfig 时钟与槽位即可。
     * 此分支不可删：位深不变时（如 LHDC 恒 32bit 容器）若跳过重配置，
     * I2S 硬件将停留在旧采样率，消费速率不匹配 → ringbuffer 积压溢出 → 蓝牙 Pkt dropped。 */
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate);

    /* Disable channel before reconfig to prevent glitches */
    if (s_i2s_out.is_started) {
        i2s_channel_disable(s_i2s_out.tx_chan);
    }

    ESP_ERROR_CHECK(i2s_channel_reconfig_std_clock(s_i2s_out.tx_chan, &clk_cfg));

    i2s_data_bit_width_t bit_width;
    switch (bit_depth) {
        case 16:
            bit_width = I2S_DATA_BIT_WIDTH_16BIT;
            break;
        case 24:
            bit_width = I2S_DATA_BIT_WIDTH_24BIT;
            break;
        case 32:
            bit_width = I2S_DATA_BIT_WIDTH_32BIT;
            break;
        default:
            ESP_LOGE(TAG, "Unsupported bit depth: %d", bit_depth);
            return ESP_ERR_INVALID_ARG;
    }

    switch (mcu_i2s_format) {
        case I2S_STD_FORMAT: {
                i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bit_width,
                                                        (channels == 1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO);
                ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(s_i2s_out.tx_chan, &slot_cfg));
            }
            break;
        case I2S_LSB_FORMAT:
            ESP_LOGE(TAG, "Unsupported I2S format: %d", mcu_i2s_format);
            return ESP_FAIL;
        case I2S_MSB_FORMAT: {
                i2s_std_slot_config_t slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(bit_width,
                                                        (channels == 1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO);
                ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(s_i2s_out.tx_chan, &slot_cfg));
            }
            break;
        case I2S_PHILIPS_FORMAT: {
                i2s_std_slot_config_t slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(bit_width,
                                                        (channels == 1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO);
                ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(s_i2s_out.tx_chan, &slot_cfg));
            }
            break;
        case I2S_RIGHT_JUSTIFIED_FORMAT:
            ESP_LOGE(TAG, "Unsupported I2S format: %d", mcu_i2s_format);
            return ESP_FAIL;
        case I2S_LEFT_JUSTIFIED_FORMAT: {
                i2s_std_slot_config_t slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(bit_width,
                                                        (channels == 1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO);
                ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(s_i2s_out.tx_chan, &slot_cfg));
            }
            break;
        case I2S_PCM: {
                i2s_std_slot_config_t slot_cfg = I2S_STD_PCM_SLOT_DEFAULT_CONFIG(bit_width,
                                                        (channels == 1) ? I2S_SLOT_MODE_MONO : I2S_SLOT_MODE_STEREO);
                ESP_ERROR_CHECK(i2s_channel_reconfig_std_slot(s_i2s_out.tx_chan, &slot_cfg));
            }
            break;
        default:
            ESP_LOGE(TAG, "Unsupported I2S format: %d", mcu_i2s_format);
            return ESP_ERR_INVALID_ARG;
    }

    /* 采样率切换时旧数据格式不匹配，必须清空 ringbuffer 避免不连续噪声 */
    size_t rb_item_size2;
    char *rb_item2;
    while ((rb_item2 = (char *)xRingbufferReceive(s_i2s_out.ringbuf, &rb_item_size2, 0)) != NULL) {
        vRingbufferReturnItem(s_i2s_out.ringbuf, rb_item2);
    }

    // 不在此处 enable，等待 audio_output_i2s_start 统一启用

    // update params
    s_i2s_out.sample_rate = sample_rate;
    s_i2s_out.bit_depth = bit_depth;
    s_i2s_out.channels = channels;
    s_i2s_out.mcu_i2s_format = mcu_i2s_format;

    ESP_LOGI(TAG, "I2S configured (not enabled, waiting for start): %dHz | %dbit | %dch",
             sample_rate, bit_depth, channels);
    return ESP_OK;
}

size_t audio_output_i2s_write(const uint8_t *pcm_data, size_t pcm_len)
{
    /* 真实写入 ringbuffer 字节速率诊断（2026-08-18） */
    static uint32_t s_wr = 0;
    static uint32_t s_w_t0 = 0;
    uint32_t now2 = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_wr += (uint32_t)pcm_len;
    if (now2 - s_w_t0 >= 2000) {
        ESP_LOGD(TAG, "RB WRITE: %u bytes/2s", (unsigned)s_wr); // test OK
        s_wr = 0;
        s_w_t0 = now2;
    }
    if (!s_i2s_out.is_initialized || !s_i2s_out.is_started) {
        return 0;
    }

    size_t item_size = 0;
    BaseType_t done = pdFALSE;

    // 在DROPPING模式下，检查ringbuffer是否已降到安全水位以下
    if (s_ringbuffer_mode == RINGBUFFER_MODE_DROPPING) {
        vRingbufferGetInfo(s_i2s_out.ringbuf, NULL, NULL, NULL, NULL, &item_size);
        // 使用更低的阈值恢复，避免频繁切换
        if (item_size <= (RINGBUF_PREFETCH_WATER_LEVEL * 3 / 4)) {
            ESP_LOGD(TAG, "ringbuffer data decreased! mode changed: RINGBUFFER_MODE_PROCESSING");
            s_ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
        } else {
            // 仍然满，继续丢弃数据
            static uint32_t drop_count = 0;
            if (++drop_count % 100 == 0) {
                ESP_LOGW(TAG, "ringbuffer is full, dropped %lu packets", drop_count);
            }
            return 0;
        }
    }

    /**
     * 注意到本地播放时，如果继续采用非阻塞式写入，会频繁触发ringbuffer overflow，改成非阻塞就没问题
     * 参考ESP32-audioI2S库，使用阻塞式写入：当ringbuffer满时等待空间，自然节流
     * timeout = 50ms，足以覆盖任何chunk的消费时间
     */
    done = xRingbufferSend(s_i2s_out.ringbuf, (void *)pcm_data, pcm_len, pdMS_TO_TICKS(200));
    if (done) {
        s_last_real_write_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }

    if (!done) {
        // ringbuffer已满且timeout到期，切换到DROPPING模式
        ESP_LOGW(TAG, "ringbuffer overflowed! mode changed: RINGBUFFER_MODE_DROPPING");
        s_ringbuffer_mode = RINGBUFFER_MODE_DROPPING;
    }

    // 在PREFETCHING模式下，检查是否达到预取阈值
    if (s_ringbuffer_mode == RINGBUFFER_MODE_PREFETCHING) {
        vRingbufferGetInfo(s_i2s_out.ringbuf, NULL, NULL, NULL, NULL, &item_size);
        if (item_size >= RINGBUF_PREFETCH_WATER_LEVEL) {
            ESP_LOGD(TAG, "ringbuffer data increased! mode changed: RINGBUFFER_MODE_PROCESSING");
            s_ringbuffer_mode = RINGBUFFER_MODE_PROCESSING;
            if (pdFALSE == xSemaphoreGive(s_i2s_out.write_semaphore)) {
                ESP_LOGE(TAG, "semaphore give failed");
            }
        }
    }

    return done ? pcm_len : 0;
}

void audio_output_i2s_start(void)
{
    if (!s_i2s_out.is_initialized) {
        ESP_LOGE(TAG, "I2S output not initialized");
        return;
    }

    if (s_i2s_out.is_started) {
        /* 合法重复调用（如封面播放器/重连/切歌路径并发触发）——幂等返回，降为调试级避免刷屏 */
        ESP_LOGD(TAG, "I2S output already started");
        return;
    }

    if (s_i2s_mutex != NULL) {
        xSemaphoreTake(s_i2s_mutex, portMAX_DELAY);
    }
    ESP_ERROR_CHECK(i2s_channel_enable(s_i2s_out.tx_chan));
    s_i2s_out.is_started = true;
    /* 锁内冲刷 DMA 残留（2026-08-16）：disable 期间 DMA 保留旧歌数据，enable 后
     * 先播残留（切歌残留源）——写静音覆盖整个 DMA 缓冲后再释放锁让 i2s_task
     * 写新歌数据（i2s_task 写 DMA 需持同一把锁，锁内冲刷无竞态）。 */
    {
        size_t flushed = 0;
        i2s_channel_write(s_i2s_out.tx_chan, s_dma_flush_silence, sizeof(s_dma_flush_silence),
                          &flushed, pdMS_TO_TICKS(200));
        size_t rb_free = 0, rb_items = 0;
        vRingbufferGetInfo(s_i2s_out.ringbuf, NULL, NULL, NULL, &rb_free, &rb_items);
        ESP_LOGD(TAG, "start flush: wrote %u/%u bytes, rb free=%u items=%u",
                  (unsigned)flushed, (unsigned)sizeof(s_dma_flush_silence),
                  (unsigned)rb_free, (unsigned)rb_items);
    }
    if (s_i2s_mutex != NULL) {
        xSemaphoreGive(s_i2s_mutex);
    }

    /* 唤醒消费任务：stop 后任务已阻塞在信号量上，仅靠 write 攒够预取水位给信号量
     * 在 ringbuffer 已空时依赖首次 PREFETCHING 阈值，存在时序竞态。这里显式给一次，
     * 任务被唤醒后立即处理（ringbuffer 为空则走 underflow 归零分支，无副作用）。 */
    if (pdFALSE == xSemaphoreGive(s_i2s_out.write_semaphore)) {
        ESP_LOGD(TAG, "write_semaphore already given, task may be busy");
    }

    ESP_LOGI(TAG, "I2S output started");
}

void audio_output_i2s_stop(void)
{
    if (!s_i2s_out.is_initialized) {
        return;
    }

    if (s_i2s_mutex != NULL) {
        xSemaphoreTake(s_i2s_mutex, portMAX_DELAY);
    }

    if (s_i2s_out.is_started) {
        /* 冲刷 TX DMA 缓冲：disable 前先写入一段静音，把 DMA 中残留的上一首歌 PCM
         * 数据挤出发送窗口。否则 disable 后 DMA 保留旧数据，下次 enable 时按新采样率
         * 播放残留，导致手动切歌瞬间出现上一首歌的变调音频（自动切歌时旧歌自然播完、
         * DMA 已空，因此没有此问题）。 */
        {
            /* 覆盖整个 DMA 缓冲的静音（240 frames * 2ch * 2byte × 4 descs）。
             * 超时 200ms：44.1k 下 960 帧需 21.8ms 消费，原 20ms 会部分写入
             * （DMA 满时只写一部分）→ 残留旧数据 → 切歌残留源。 */
            size_t flush_written = 0;
            i2s_channel_write(s_i2s_out.tx_chan, s_dma_flush_silence, sizeof(s_dma_flush_silence),
                              &flush_written, pdMS_TO_TICKS(200));
            ESP_LOGD(TAG, "stop flush: wrote %u/%u bytes", (unsigned)flush_written,
                      (unsigned)sizeof(s_dma_flush_silence));
        }

        ESP_ERROR_CHECK(i2s_channel_disable(s_i2s_out.tx_chan));
        s_i2s_out.is_started = false;
    }

    /* 无论是否 started：清空 ringbuffer + 归零频谱。
     * 否则（如停止前 I2S 未启动）ringbuffer 保留旧歌 PCM、频谱保留旧值，
     * 重新播放/挂载后出现残留音频，熄屏频谱显示残留。 */
    if (s_i2s_out.ringbuf != NULL) {
        size_t item_size;
        char *item;
        uint32_t drain_cnt = 0;
        while ((item = (char *)xRingbufferReceive(s_i2s_out.ringbuf, &item_size, 0)) != NULL) {
            vRingbufferReturnItem(s_i2s_out.ringbuf, item);
            drain_cnt++;
        }
        ESP_LOGD(TAG, "stop drained %lu ringbuffer items", (unsigned long)drain_cnt);
    }

    /* 重置模式为 PREFETCHING：write 只在 PREFETCHING 模式攒够预取水位后 give 信号量。
     * 停止时若模式残留 PROCESSING/DROPPING（播放稳定期/ringbuffer 曾满），重新播放时
     * write 将永远不 give 信号量，消费任务阻塞在 xSemaphoreTake(portMAX_DELAY) 无法唤醒，
     * 表现为"播放进度正常但完全无声音"（含切歌）。 */
    s_ringbuffer_mode = RINGBUFFER_MODE_PREFETCHING;


    if (s_i2s_mutex != NULL) {
        xSemaphoreGive(s_i2s_mutex);
    }

    ESP_LOGI(TAG, "I2S output stopped");
}

uint32_t audio_output_i2s_get_starved_ms(void)
{
    return s_i2s_starved_ms;
}

uint32_t audio_output_i2s_get_buffer_level(void)
{
    if (!s_i2s_out.is_initialized || s_i2s_out.ringbuf == NULL) {
        return 0;
    }
    size_t item_size = 0;
    vRingbufferGetInfo(s_i2s_out.ringbuf, NULL, NULL, NULL, NULL, &item_size);
    return (uint32_t)item_size;
}

uint32_t audio_output_i2s_get_buffer_size(void)
{
    return RINGBUF_SIZE;
}

uint32_t audio_output_i2s_get_sample_rate(void)
{
    return s_i2s_out.is_initialized ? s_i2s_out.sample_rate : 0;
}

uint8_t audio_output_i2s_get_bit_depth(void)
{
    return s_i2s_out.is_initialized ? s_i2s_out.bit_depth : 0;
}

uint8_t audio_output_i2s_get_channels(void)
{
    return s_i2s_out.is_initialized ? s_i2s_out.channels : 0;
}

esp_err_t audio_output_init(void) {
    ESP_LOGI(TAG, "Init I2S and DAC");
    
    // 先清零结构体，避免脏数据
    memset(&s_i2s_out, 0, sizeof(audio_output_i2s_t));

    s_i2s_out.sample_rate = CONFIG_DEFAULT_I2S_SAMPLE_RATE;
    s_i2s_out.bit_depth = CONFIG_DEFAULT_I2S_BIT_DEPTH;
    s_i2s_out.channels = CONFIG_DEFAULT_I2S_CHANNELS;
    s_i2s_out.system_volume = DEFAULT_SYSTEM_VOLUME;
    s_i2s_out.gain = 1.0f;
    s_i2s_out.mcu_i2s_mode = I2S_ROLE_MASTER;
    s_i2s_out.mcu_i2s_format = I2S_STD_FORMAT;

    // debug only

    // Initialize I2S with 32-bit to pre-allocate enough DMA buffer for all codecs
    // When switching to 16-bit later, the DMA buffer will be large enough
    ESP_ERROR_CHECK(audio_output_i2s_init(s_i2s_out.sample_rate, 32, s_i2s_out.channels, 
                                            s_i2s_out.mcu_i2s_format, s_i2s_out.mcu_i2s_mode));
    // Now configure to actual bit depth
    ESP_ERROR_CHECK(audio_output_i2s_configure(s_i2s_out.sample_rate, s_i2s_out.bit_depth, s_i2s_out.channels, s_i2s_out.mcu_i2s_format));
    ESP_ERROR_CHECK(audio_output_set_volume(s_i2s_out.system_volume));
    // 初始化 DSP EQ（全局单例，后续由 I2S 任务在处理音频数据时调用）
    return ESP_OK;
}

esp_err_t audio_output_set_volume(uint8_t volume)
{
    if (volume > SYSTEM_VOLUME_MAX) {
        volume = SYSTEM_VOLUME_MAX;
    }
    /* 保存系统音量（0-127） */
    s_i2s_out.system_volume = volume;
    /* 软件音量：功率曲线 (v/127)^2，人耳感知平滑；
     * 低音量段细腻、全程均匀可调 */
    float v = (float)volume / 127.0f;
    s_i2s_out.gain = v * v;
    s_i2s_out.dac_volume = (uint8_t)(s_i2s_out.gain * 100.0f);
    return ESP_OK;
}

uint8_t audio_output_get_volume(void)
{
    return s_i2s_out.system_volume;
}

uint8_t audio_output_get_dac_volume(void)
{
    return s_i2s_out.dac_volume;
}


