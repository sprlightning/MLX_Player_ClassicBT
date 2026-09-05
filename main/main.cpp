/************************************
 * MLX_Player_ClassicBT - main.cpp
 * 硬件：ESP-32S-N4R8核心板(ESP32-D0WD，4MB Flash，8MB PSRAM) + PCM5102A
 * 环境：vscode-esp-idf v5.1.4，已集成A2DP extended codecs，不使用arduino-esp32组件
 * 功能：A2DP接收音频并通过I2S输出音频，同时通过串口输出歌词
 * 使用的GitHub库：
 * ESP32-A2DP库：https://github.com/pschatzmann/ESP32-A2DP
 * 音频工具库：https://github.com/pschatzmann/arduino-audio-tools
 * 音频工具使用教程：https://github.com/pschatzmann/arduino-audio-tools/wiki/Encoding-and-Decoding-of-Audio
 *************************************/

#include "BluetoothA2DPSinkQueued.h"
#include "driver/i2c.h"
#include "driver/i2s_std.h"
#include "nvs_flash.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>
#include <vector>
#include <string>
#include <dirent.h>

// 音频工具相关头文件
#define ARDUINO_AUDIO_TOOLS_NO_NET_MACROS 1
#include "AudioTools.h"

// 硬件定义
#define SPI_CLK         (GPIO_NUM_14)
#define SPI_MISO        (GPIO_NUM_2)
#define SPI_MOSI        (GPIO_NUM_15)
#define SPI_CS          (GPIO_NUM_13)

#define I2C_NUM         (I2C_NUM_0)
#define I2C_SCL         (GPIO_NUM_23)
#define I2C_SDA         (GPIO_NUM_18)
#define I2C_FREQ        (400000)

#define I2S_PORT_NUM    (I2S_NUM_0)
// #define I2S_MCK         (GPIO_NUM_0) // ESP32 only support to set GPIO0/GPIO1/GPIO3 as mclk signal
#define I2S_BCK         (GPIO_NUM_2)
#define I2S_WS          (GPIO_NUM_15)
#define I2S_DO          (GPIO_NUM_14)
// #define I2S_DI          (GPIO_NUM_15)

// 其他定义
static const char *TAG = "main";
const char *bt_name = "A2DP_Sink_Player";
#define DEFAULT_AVRC_VOLUME (26)

// 系统状态定义
typedef enum {
    STATE_POWER_OFF,
    STATE_WELCOME,
    STATE_MAIN_MENU,
    STATE_BLUETOOTH_PLAY,
    STATE_SD_PLAY,
    STATE_SETTINGS,
    STATE_SHUTDOWN_CONFIRM
} SystemState;

// 播放模式等其他枚举定义
typedef enum {
    A2DP_SINK = 0,
    SD_MODE = 1
} PlayMode;

typedef enum {
    MENU_BLUETOOTH,
    MENU_SD_CARD,
    MENU_SETTINGS,
    MENU_COUNT
} MenuOption;

typedef enum {
    KEY_IDLE,
    KEY_PRESSED,
    KEY_SHORT_PRESS,
    KEY_LONG_PRESS,
    KEY_RELEASED
} KeyState;

// 系统全局变量
SystemState current_state = STATE_POWER_OFF;
PlayMode play_mode = A2DP_SINK;
MenuOption current_menu = MENU_BLUETOOTH;
KeyState encoder_key_state = KEY_IDLE;
unsigned long encoder_key_press_time = 0;
const unsigned long SHORT_PRESS_THRESHOLD = 30;
const unsigned long LONG_PRESS_THRESHOLD = 1000;
long encoder_position = 0;
long last_encoder_position = 0;
bool encoder_scrolled = false;
#define SCREEN_TIMEOUT_MIN 5
#define SCREEN_TIMEOUT_MAX 60
uint8_t screen_timeout = 30;
unsigned long last_user_activity = 0;
bool screen_on = true;
bool encoder_rotated_while_pressed = false; // 记录按键按下期间是否有转动

// 歌曲元数据结构
typedef struct {
    char track_title[256];
    char title[256];
    char artist[256];
    char album[256];
    char genre[256];
    char track_num[32];
} SongMetadata;

static SongMetadata current_metadata = {0};
static SongMetadata last_metadata = {0};
static bool is_new_song = true;
static bool is_metadata_complete = false;

// 音频参数配置
static uint32_t current_sample_rate = 48000;
static uint8_t current_bit_depth = 32;
static uint8_t current_channels = 2;
static uint8_t avrc_vol = DEFAULT_AVRC_VOLUME;
static bool encoder_software_lock = false;
static const char* current_codec_name = ">B<";
static uint32_t audio_file_bit_rate = 0;
static FILE* g_current_file = NULL; // 定义全局文件指针

#define get_local_vol() avrc_vol
static esp_a2d_audio_state_t current_audio_state = ESP_A2D_AUDIO_STATE_STOPPED;

// 编码器GPIO定义
// #define ENCODER_A       (GPIO_NUM_32)
// #define ENCODER_B       (GPIO_NUM_33)
// #define ENCODER_BUTTON  (GPIO_NUM_27)
// #define BUTTON1         (GPIO_NUM_36)
// #define BUTTON2         (GPIO_NUM_39)
#define VOLUME_STEP     (2)
#define DAC_VOLUME_MIN      (0)
#define DAC_VOLUME_MAX      (90)
#define AVRC_VOLUME_MIN     (0)
#define AVRC_VOLUME_MAX     (127)

int last_processed_volume = -1;
unsigned long last_volume_time = 0;
const int VOLUME_THRESHOLD = 5;
const unsigned long DEBOUNCE_DELAY = 200;
bool is_a2dp_started = false;

// 内存卡相关定义
#define SD_D0   (GPIO_NUM_2)
#define SD_D1   (GPIO_NUM_4)
#define SD_D2   (GPIO_NUM_12)
#define SD_D3   (GPIO_NUM_13)
#define SD_CLK  (GPIO_NUM_14)
#define SD_CMD  (GPIO_NUM_15)
#define SD_DET  (GPIO_NUM_34)

// 内部函数声明（暂无）


// 全局音频对象
I2SStream i2s_out; // final output of decoded stream
BluetoothA2DPSinkQueued a2dp_sink(i2s_out);

// 范围约束函数
uint8_t constrain(uint8_t x, uint8_t min_val, uint8_t max_val) {
    if (x < min_val) return min_val;
    if (x > max_val) return max_val;
    return x;
}

// SPI初始化
static esp_err_t spi_init(void) {
    spi_bus_config_t bus_config = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 0,
    };
    
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SPI初始化失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    gpio_reset_pin(SPI_CS);
    gpio_set_direction(SPI_CS, GPIO_MODE_OUTPUT);
    gpio_set_level(SPI_CS, 1);
    
    ESP_LOGI(TAG, "SPI初始化完成");
    return ret;
}

// I2C初始化
static esp_err_t i2c_init(i2c_port_t port, uint32_t freq, uint8_t i2c_sda, uint8_t i2c_scl) {
    i2c_config_t i2c_cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_sda,
        .scl_io_num = i2c_scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = freq,
        },
    };
    
    esp_err_t ret = i2c_param_config(port, &i2c_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C参数配置失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = i2c_driver_install(port, i2c_cfg.mode, 0, 0, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C驱动安装失败: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C初始化完成, port: %d, freq: %dHz", port, freq);
    return ret;
}

// 修正I2S初始化函数
static esp_err_t i2s_init(uint8_t port, uint32_t sample_rate, uint32_t _bits_per_sample, uint32_t _channels) {
    if (sample_rate <= 0 || _bits_per_sample <= 0) return ESP_FAIL;

    ESP_LOGI(TAG, "准备初始化I2S");

    // 确保I2S资源已释放
    if (i2s_out.isActive()) {
        i2s_out.end();
        vTaskDelay(pdMS_TO_TICKS(100));
        // 额外的硬件重置
        gpio_set_level(I2S_BCK, 0);
        gpio_set_level(I2S_WS, 0);
        gpio_set_level(I2S_DO, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // 硬件重置
    gpio_reset_pin(I2S_BCK);
    gpio_reset_pin(I2S_WS);
    gpio_reset_pin(I2S_DO);
    vTaskDelay(pdMS_TO_TICKS(10));

    I2SConfig i2s_cfg;
    i2s_cfg.channels = _channels;
    i2s_cfg.sample_rate = sample_rate;
    i2s_cfg.bits_per_sample = _bits_per_sample;
    i2s_cfg.rx_tx_mode = TX_MODE;
    i2s_cfg.i2s_format = I2S_PHILIPS_FORMAT;
    i2s_cfg.signal_type = Digital;
    i2s_cfg.is_master = true; 
    i2s_cfg.port_no = I2S_PORT_NUM;
    i2s_cfg.pin_ws = I2S_WS;
    i2s_cfg.pin_bck = I2S_BCK;
    i2s_cfg.pin_data = I2S_DO;
    i2s_cfg.pin_data_rx = -1;
    i2s_cfg.pin_mck = -1;
    i2s_cfg.buffer_count = 3; // 推荐3
    i2s_cfg.buffer_size = 1024; // 推荐1024
    i2s_cfg.use_apll = true;
    i2s_cfg.auto_clear = true;
    i2s_cfg.channel_format = I2SChannelSelect::Default;
    i2s_cfg.mclk_multiple = -1;

    if (!i2s_out.begin(i2s_cfg)) {
        ESP_LOGE(TAG, "I2S初始化失败");
        return ESP_FAIL;
    }

    size_t dma_total_size = i2s_cfg.buffer_count * i2s_cfg.buffer_size;
    int frame_size = i2s_cfg.bits_per_sample * i2s_cfg.channels / 8;
    size_t total_frames = dma_total_size / frame_size;
    float buffer_duration = (float)total_frames / i2s_cfg.sample_rate;

    ESP_LOGI(TAG, "I2S init success, port%d | %ldHz | %dbit | %dchs | DMA: %dbytes | %.2fms", 
        port, sample_rate, _bits_per_sample, _channels, dma_total_size, buffer_duration * 1000);
    return ESP_OK;
}

// 音频状态回调函数
void audio_state_changed(esp_a2d_audio_state_t state, void *user_data) {
    static esp_a2d_audio_state_t last_state = ESP_A2D_AUDIO_STATE_STOPPED;
    current_audio_state = state;

    if (last_state != state) {
        if (state == ESP_A2D_AUDIO_STATE_STARTED) {
            ESP_LOGI(TAG, "切歌成功，正在播放");
        }
        last_state = state;
    }

    switch (state) {
        case ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND: 
            ESP_LOGI(TAG, "音频已暂停"); 
            break;
        case ESP_A2D_AUDIO_STATE_STOPPED: 
            ESP_LOGI(TAG, "音频已停止"); 
            break;
        case ESP_A2D_AUDIO_STATE_STARTED: 
            ESP_LOGI(TAG, "音频已开始"); 
            break;
        default: 
            ESP_LOGI(TAG, "音频状态: %d", state); 
            break;
    }
}

// 连接状态回调函数
void connection_state_changed(esp_a2d_connection_state_t state, void *user_data) {
    bool connected = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);
    switch (state) {
        case ESP_A2D_CONNECTION_STATE_DISCONNECTED: 
            ESP_LOGI(TAG, "bt连接状态: 已断开"); 
            if (current_state == STATE_BLUETOOTH_PLAY) {
                printf("请连接蓝牙\n");
            }
            break;
        case ESP_A2D_CONNECTION_STATE_CONNECTING: 
            ESP_LOGI(TAG, "bt连接状态: 正在连接"); 
            if (current_state == STATE_BLUETOOTH_PLAY) {
                printf("A2DP协商中，请等待...\n");
            }
            break;
        case ESP_A2D_CONNECTION_STATE_CONNECTED: 
            ESP_LOGI(TAG, "bt连接状态: 已连接"); 
            if (a2dp_sink.is_avrc_connected()) {
                ESP_LOGI(TAG, "AVRC连接成功，支持音量同步");
            } else {
                ESP_LOGW(TAG, "AVRC连接失败，音量同步不可用");
            }
            break;
        case ESP_A2D_CONNECTION_STATE_DISCONNECTING: 
            ESP_LOGI(TAG, "bt连接状态: 正在断开"); 
            break;
        default: 
            ESP_LOGI(TAG, "bt连接状态: 未知状态"); 
            break;
    }
}

// 判断两首歌的元数据是否相同
bool is_metadata_equal(const SongMetadata* a, const SongMetadata* b) {
    return (strcmp(a->album, b->album) == 0);
}

// 元数据回调函数
void avrc_metadata_callback(uint8_t attribute_id, const uint8_t *metadata) {
    if (metadata == NULL) return;

    switch (attribute_id) {
        case ESP_AVRC_MD_ATTR_TITLE:
            // 注意这里才是元数据更新后实时更新的地方
            if (is_new_song) {
                // 首次传输：作为歌曲标题存入track_title
                strncpy(current_metadata.track_title, (char*)metadata, sizeof(current_metadata.track_title)-1);
                current_metadata.track_title[sizeof(current_metadata.track_title)-1] = '\0';
                ESP_LOGI(TAG, "歌曲标题: %s", current_metadata.track_title);
                // 这里显示歌曲标题（占用第2-3页）
                if (strlen(current_metadata.track_title) > 0) {
                    printf("%s\n", current_metadata.track_title);
                } else {
                    printf("未知曲目\n");
                }
            } else {
                // 后续传输：作为歌词存入title
                strncpy(current_metadata.title, (char*)metadata, sizeof(current_metadata.title)-1);
                current_metadata.title[sizeof(current_metadata.title)-1] = '\0';
                // ESP_LOGI(TAG, "歌词: %s", current_metadata.title);
                // 在串口打印歌词
                if (strlen(current_metadata.title) > 0) {
                    printf("%s\n", current_metadata.title);
                }
            }
            // 更新播放/暂停符号
            if (current_state == STATE_BLUETOOTH_PLAY && current_audio_state == ESP_A2D_AUDIO_STATE_STARTED) {
                printf("⏸\n"); // 显示播放状态
            } else if (current_state == STATE_BLUETOOTH_PLAY && 
                (current_audio_state == ESP_A2D_AUDIO_STATE_REMOTE_SUSPEND || 
                current_audio_state == ESP_A2D_AUDIO_STATE_STOPPED)){
                    printf("▶\n"); // 显示暂停状态
            }
            break;
        case ESP_AVRC_MD_ATTR_ARTIST:
            strncpy(current_metadata.artist, (char*)metadata, sizeof(current_metadata.artist)-1);
            current_metadata.artist[sizeof(current_metadata.artist)-1] = '\0';
            break;
        case ESP_AVRC_MD_ATTR_ALBUM:
            strncpy(current_metadata.album, (char*)metadata, sizeof(current_metadata.album)-1);
            current_metadata.album[sizeof(current_metadata.album)-1] = '\0';
            break;
        case ESP_AVRC_MD_ATTR_GENRE:
            strncpy(current_metadata.genre, (char*)metadata, sizeof(current_metadata.genre)-1);
            current_metadata.genre[sizeof(current_metadata.genre)-1] = '\0';
            break;
        case ESP_AVRC_MD_ATTR_TRACK_NUM:
            strncpy(current_metadata.track_num, (char*)metadata, sizeof(current_metadata.track_num)-1);
            current_metadata.track_num[sizeof(current_metadata.track_num)-1] = '\0';
            is_metadata_complete = true;
            break;
        default:
            return;
    }

    if (is_metadata_complete) {
        if (!is_metadata_equal(&current_metadata, &last_metadata)) {
            is_new_song = true; // 标记为新歌
            ESP_LOGI(TAG, "====== 发现歌曲 ======");
            ESP_LOGI(TAG, "标题: %s | 艺术家: %s | 专辑: %s | 流派: %s | 曲目号: %s", 
                current_metadata.track_title, current_metadata.artist, current_metadata.album, current_metadata.genre, current_metadata.track_num);
            ESP_LOGI(TAG, "======================");
            
            memcpy(&last_metadata, &current_metadata, sizeof(SongMetadata));
        } else {
            is_new_song = false; // 同一首歌，后续传输为歌词
        }
        is_metadata_complete = false;
    }
}

// codec配置回调函数
void codec_config_callback(uint32_t rate, uint8_t bps, uint8_t channels, const char* codec_name) {
    esp_a2d_audio_state_t prev_state = current_audio_state;
    ESP_LOGI(TAG, "切换codec前A2DP_SINK的状态: %s", (prev_state == 0) ? "暂停/停止" : "播放");

    current_sample_rate = rate;
    current_bit_depth = bps;
    current_channels = channels;
    current_codec_name = codec_name;

    ESP_LOGI(TAG, "检测到A2DP音频配置变更(%s | %ldHz | %dbit | %dchs), 准备同步到codec", 
        codec_name, rate, bps, channels);

    a2dp_sink.pause();
    vTaskDelay(pdMS_TO_TICKS(20));

    if (prev_state == ESP_A2D_AUDIO_STATE_STARTED) a2dp_sink.pause();
    vTaskDelay(pdMS_TO_TICKS(30));
    i2s_out.end();
    vTaskDelay(pdMS_TO_TICKS(30));
    i2s_init(I2S_PORT_NUM, current_sample_rate, current_bit_depth, current_channels);

    if (current_sample_rate >= 88200) {
        a2dp_sink.set_i2s_ringbuffer_prefetch_percent(75);
    } else if (current_sample_rate >= 48000) {
        a2dp_sink.set_i2s_ringbuffer_prefetch_percent(65);
    } else {
        a2dp_sink.set_i2s_ringbuffer_prefetch_percent(55);
    }

    if (current_state == STATE_BLUETOOTH_PLAY) {
        printf("%s | %s\n", current_metadata.title, current_metadata.artist);
        printf("%s | %s | %s | %s | %s\n", current_sample_rate, current_bit_depth, current_codec_name, get_local_vol());
    }

    if (prev_state == ESP_A2D_AUDIO_STATE_STARTED) a2dp_sink.play();
}

// I2C设备扫描函数
static esp_err_t i2c_scan(i2c_port_t port) {
    ESP_LOGI(TAG, "在port%d扫描I2C设备...", port);
    for (uint8_t addr = 0x08; addr < 0x77; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(port, cmd, 100 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "发现I2C设备(写地址): 0x%02X", addr << 1);
        }
    }
    return ESP_OK;
}

// 重置元数据
void reset_metadata() {
    memset(&current_metadata, 0, sizeof(SongMetadata));
    is_new_song = true;
    is_metadata_complete = false;
}

// 音量变化回调函数，添加screen_on检查
void volume_change_callback(int vol) {
    
    uint8_t new_vol = (vol < AVRC_VOLUME_MIN) ? AVRC_VOLUME_MIN :
                      (vol > AVRC_VOLUME_MAX) ? AVRC_VOLUME_MAX : vol;
    
    avrc_vol = new_vol;
    ESP_LOGI(TAG, "A2DP Source vol: %d → 本地avrc_vol同步为%d", vol, avrc_vol);
}

extern "C" void app_main(void) {
    esp_log_level_set("BT_API", ESP_LOG_WARN);
    esp_log_level_set("BT_AV", ESP_LOG_WARN);
    esp_log_level_set("gpio", ESP_LOG_WARN);
    esp_log_level_set("audio-tools", ESP_LOG_WARN); // 屏蔽StreamCopy时的大量常规copy日志
    esp_log_level_set("libhelix", ESP_LOG_WARN); // 屏蔽mp3解码时的大量常规日志

    esp_task_wdt_deinit();

    esp_task_wdt_config_t twdt_config = {
        .timeout_ms = 15000,
        .idle_core_mask = (1 << 0) | (1 << 1),
        .trigger_panic = true,
    };
    esp_task_wdt_init(&twdt_config);

    esp_err_t ret_buf[10] = {0};

    ret_buf[0] = nvs_flash_init();
    if (ret_buf[0] == ESP_ERR_NVS_NO_FREE_PAGES || ret_buf[0] == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret_buf[1] = nvs_flash_init();
        ESP_LOGI("NVS", "Flash-NVS分区已重新初始化");
    }

    ret_buf[2] = spi_init();
    ret_buf[3] = i2c_init(I2C_NUM, I2C_FREQ, I2C_SDA, I2C_SCL);
    ret_buf[4] = i2c_scan(I2C_NUM);
    ret_buf[5] = i2s_init(I2S_PORT_NUM, current_sample_rate, current_bit_depth, current_channels);

    for (int i = 0; i < 10; i++) {
        if (ret_buf[i] != ESP_OK) {
            ESP_LOGI(TAG, "Operation ret[%d] failed: %s", i, esp_err_to_name(ret_buf[i]));
            return;
        }
    }

    ESP_LOGI(TAG, "ESP32初始化完成，欢迎使用！");

    a2dp_sink.set_on_connection_state_changed(connection_state_changed);
    
    a2dp_sink.set_avrc_metadata_attribute_mask(
        ESP_AVRC_MD_ATTR_TITLE |
        ESP_AVRC_MD_ATTR_ARTIST |
        ESP_AVRC_MD_ATTR_ALBUM |
        ESP_AVRC_MD_ATTR_GENRE |
        ESP_AVRC_MD_ATTR_TRACK_NUM
    );
    a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
    a2dp_sink.set_on_audio_state_changed(audio_state_changed);
    a2dp_sink.set_codec_config_callback(codec_config_callback);

    a2dp_sink.set_avrc_rn_events({ESP_AVRC_RN_VOLUME_CHANGE});

    a2dp_sink.set_avrc_connection_state_callback([](bool connected) {
        if (connected) {
            ESP_LOGI(TAG, "AVRC连接成功，支持双向音量控制");
        } else {
            ESP_LOGW(TAG, "AVRC连接断开，音量同步失效");
        }
    });

    a2dp_sink.set_avrc_rn_volumechange(volume_change_callback);

    a2dp_sink.set_avrc_rn_volumechange_completed([](int vol) {
        ESP_LOGI(TAG, "Source vol: %d", vol);
    });

    a2dp_sink.set_auto_reconnect(true);
    a2dp_sink.set_task_core(1);
    a2dp_sink.set_event_queue_size(30);
    a2dp_sink.set_event_stack_size(4096);
    a2dp_sink.set_i2s_stack_size(4096);
    a2dp_sink.set_i2s_task_priority(configMAX_PRIORITIES - 4);

    a2dp_sink.set_i2s_ringbuffer_size(16384);
    a2dp_sink.set_i2s_write_size_upto(1440);
    a2dp_sink.set_i2s_ticks(50);

    a2dp_sink.start(bt_name);
    is_a2dp_started = true;
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (a2dp_sink.is_avrc_connected()) {
        a2dp_sink.set_volume(DEFAULT_AVRC_VOLUME);
    }

    if (!a2dp_sink.is_avrc_connected()) {
        ESP_LOGW(TAG, "AVRC not ready...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
}

