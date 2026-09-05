#ifndef AUDIO_OUTPUT_I2S_H
#define AUDIO_OUTPUT_I2S_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2s_std.h"


#define I2S_PORT_NUM    (I2S_NUM_0)

#ifdef __cplusplus
extern "C" {
#endif

/* 系统音量（0-127），AVRCP absolute volume 同级 */
#define SYSTEM_VOLUME_MIN      (0)
#define SYSTEM_VOLUME_MAX      (127)
/* 默认系统音量 */
#define DEFAULT_SYSTEM_VOLUME  (60)

/* 软件增益百分比（0-100，PCM5102A 无硬件音量） */
#define DAC_VOLUME_MIN         (0)
#define DAC_VOLUME_MAX         (100)

typedef enum {
    I2S_STD_FORMAT, // 标准I2S格式，等效于飞利浦格式(LRCK低电平=左声道；高电平=右声道；收发端位数可以不同)
    I2S_LSB_FORMAT, // LSB低位优先格式，等效于右对齐格式(LRCK高电平=左声道，低电平=右声道；声道情况左右对齐格式都一样，且和标准I2S格式相反)
    I2S_MSB_FORMAT, // MSB高位优先格式，等效于左对齐格式(LRCK高电平=左声道，低电平=右声道，时钟周期足够长便能支持16到32bit)
    I2S_PHILIPS_FORMAT, // 飞利浦I2S格式，等效于标准I2S格式
    I2S_RIGHT_JUSTIFIED_FORMAT, // 右对齐格式，等效于LSB低位优先格式
    I2S_LEFT_JUSTIFIED_FORMAT, // 左对齐格式，等效于MSB高位优先格式
    I2S_PCM, // I2S PCM标准格式(左右声道不固定，帧起始0延迟，固定50%占空比，通常是高位先行)
} mcu_i2s_fmt_t;

/**
 * @brief 初始化I2S输出
 *
 * @param sample_rate 采样率
 * @param bit_depth 位深
 * @param channels 通道数
 * @param mcu_i2s_format MCU I2S格式
 * @param mcu_i2s_mode MCU I2S模式
 * @return ESP_OK 成功
 * @return ESP_FAIL 失败
 */
esp_err_t audio_output_i2s_init(uint32_t sample_rate, uint8_t bit_depth, uint8_t channels, 
                                mcu_i2s_fmt_t mcu_i2s_format, i2s_role_t mcu_i2s_mode);

/**
 * @brief 清理I2S输出
 */
void audio_output_i2s_cleanup(void);

/**
 * @brief 配置I2S输出参数
 *
 * @param sample_rate 采样率
 * @param bit_depth 位深
 * @param channels 通道数
 * @return ESP_OK 成功
 * @return ESP_FAIL 失败
 */
esp_err_t audio_output_i2s_configure(uint32_t sample_rate, uint8_t bit_depth, uint8_t channels, mcu_i2s_fmt_t mcu_i2s_format);

/**
 * @brief 写入PCM数据到I2S
 *
 * @param pcm_data PCM数据
 * @param pcm_len PCM数据长度
 * @return 写入的字节数
 */
size_t audio_output_i2s_write(const uint8_t *pcm_data, size_t pcm_len);

/**
 * @brief 启动I2S输出
 */
void audio_output_i2s_start(void);

/**
 * @brief 停止I2S输出
 */
void audio_output_i2s_stop(void);

/**
 * @brief 初始化音频输出
 */
esp_err_t audio_output_init(void);

/**
 * @brief 获取I2S ringbuffer当前数据量
 *
 * @return ringbuffer中当前的数据字节数
 */
uint32_t audio_output_i2s_get_buffer_level(void);
uint32_t audio_output_i2s_get_starved_ms(void);
uint32_t audio_output_i2s_get_starve_episodes(void);   /* ≥500ms 饥饿段累计计数 */

/**
 * @brief 获取I2S ringbuffer总大小
 *
 * @return ringbuffer总字节数
 */
uint32_t audio_output_i2s_get_buffer_size(void);

/**
 * @brief 获取I2S当前采样率
 *
 * @return 当前采样率（Hz），未初始化时返回0
 */
uint32_t audio_output_i2s_get_sample_rate(void);

/**
 * @brief 获取I2S当前位深
 *
 * @return 当前位深（bit），未初始化时返回0
 */
uint8_t audio_output_i2s_get_bit_depth(void);

/**
 * @brief 获取I2S当前通道数
 *
 * @return 当前通道数，未初始化时返回0
 */
uint8_t audio_output_i2s_get_channels(void);

/**
 * @brief 设置系统音量（0-127），经功率曲线映射为软件增益
 */
esp_err_t audio_output_set_volume(uint8_t volume);

/**
 * @brief 获取当前系统音量（0-127）
 */
uint8_t audio_output_get_volume(void);

/**
 * @brief 获取当前软件增益百分比（0-100）
 */
uint8_t audio_output_get_dac_volume(void);


#ifdef __cplusplus
}
#endif

#endif
