/*
 * MLX_Player_ClassicBT - minimal native A2DP sink
 * Hardware: ESP32-CAM-N4R8 (ESP-32S Module, ESP32-D0WD) + PCM5102A I2S DAC
 * BT/I2S use native driver (audio_output_i2s + bt_app_av).
 * esp-idf use https://github.com/sprlightning/esp-idf-bt-multiple-codecs/tree/a2dp-codecs/v5.1.4
 * Bluetooth PHY_Init may cause BROWNOUT, need to disable BROWNOUT_CHECK.
 * Wakeness PHY data may limit the power of Bluetooth, need a strong power supply.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "esp_log.h"

#include "esp_bt.h"
#include "bt_app_core.h"
#include "bt_app_av.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"

#include "audio_output_i2s.h"
#include "task_config.h"

/* event for stack up */
enum {
    BT_APP_EVT_STACK_UP = 0,
};

void app_main(void)
{
    /* initialize NVS — used to store PHY calibration data */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* 临时诊断：开机主动回连失败排查（SDP conn error 0x9）——抓 page/ACL/加密过程 */
    esp_log_level_set("BT_HCI", ESP_LOG_DEBUG);
    esp_log_level_set("BT_SDP", ESP_LOG_DEBUG);
    esp_log_level_set("BT_L2CAP", ESP_LOG_DEBUG);
    esp_log_level_set("BT_AV", ESP_LOG_DEBUG);
    esp_log_level_set("BT_BTC", ESP_LOG_DEBUG);
    esp_log_level_set("BT_APPL", ESP_LOG_DEBUG);
    esp_log_level_set("BT_AVDT", ESP_LOG_DEBUG);

    /* I2S output (PCM5102A), ringbuffer + consumer task inside */
    ESP_ERROR_CHECK(audio_output_init());

    /* Classic BT only: release BLE controller memory (BTDM controller keeps it) */
    esp_err_t mem_ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
    if (mem_ret != ESP_OK) {
        ESP_LOGW(BT_AV_TAG, "BT controller mem_release skipped: %s", esp_err_to_name(mem_ret));
    }

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if ((err = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(err));
        return;
    }
    if ((err = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(err));
        return;
    }
    if ((err = esp_bluedroid_init()) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s initialize bluedroid failed: %s", __func__, esp_err_to_name(err));
        return;
    }
    if ((err = esp_bluedroid_enable()) != ESP_OK) {
        ESP_LOGE(BT_AV_TAG, "%s enable bluedroid failed: %s", __func__, esp_err_to_name(err));
        return;
    }

#if (CONFIG_BT_SSP_ENABLED == true)
    /* default parameters for Secure Simple Pairing */
    esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
    esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
    esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

    /* legacy pairing with fixed pin 1234 */
    esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_FIXED;
    esp_bt_pin_code_t pin_code;
    pin_code[0] = '1';
    pin_code[1] = '2';
    pin_code[2] = '3';
    pin_code[3] = '4';
    esp_bt_gap_set_pin(pin_type, 4, pin_code);

    bt_app_task_start_up();
    /* device name, connection mode and profile set up */
    bt_app_work_dispatch(bt_av_hdl_stack_evt, BT_APP_EVT_STACK_UP, NULL, 0, NULL);

    ESP_LOGI(BT_AV_TAG, "BT player started");

    /* 蓝牙自动回连：开启后进入蓝牙模式即主动连接上次配对设备（像普通蓝牙耳机）。
     * 复用 BT_AV_BG_WORK_RECONNECT 专用任务（延迟 ~400ms 等 A2DP 初始化完成再
     * esp_a2d_sink_connect last_bda），与断开后恢复 AVRCP 的回连相互独立。 */
#if (CONFIG_EXAMPLE_A2DP_SINK_AUTO_RECONNECT == true)
    ESP_LOGI(BT_AV_TAG, "Auto reconnect enabled, scheduling connect to last device");
    bt_reconnect();
#endif
}
