/**
 * task_config.h - 应用任务配置（核 / 栈大小(字节) / 优先级）
 *
 * 约定：
 *  - 栈大小为字节（ESP-IDF 的 usStackDepth 单位是字节，与 vanilla FreeRTOS
 *    的 word 不同）；
 *  - 优先级 0~24（configMAX_PRIORITIES=25），数值大者优先；
 *  - 核：0=Core0，1=Core1。
 *
 * 底层蓝牙任务（只观察，勿动）：
 *   BT controller  Core0 prio 23 栈 4096（esp_task.h 硬编码）
 *   hciT           Core1 prio 22 栈 2560（hci_layer.c 硬编码）
 *   BTU_TASK       Core1 prio 20 栈 8704（CONFIG_BT_BTU_TASK_STACK_SIZE）
 *   BTC_TASK       Core1 prio 19 栈 16384+512（CONFIG_BT_BTC_TASK_STACK_SIZE）
 *   A2DP_DECODER   Core1 prio 19 栈 50KB(SPIRAM)（btc_a2dp_sink.c 硬编码，
 *                  音频解码发生在此任务上下文，esp_a2d_sink_data_cb 也在其中）
 *   警示：Core1 上 19/20/22 协议栈任务常驻，应用层任务放 Core1 时优先级
 *   必须 < 19；放 Core0 的应用任务同样不得超过 controller 的 23。
 */
#ifndef __TASK_CONFIG_H
#define __TASK_CONFIG_H

#include "freertos/FreeRTOS.h"   /* configMAX_PRIORITIES */

/* ============ 音频 ============ */
/* i2s_task：I2S 持续消费 ringbuffer 音频。Core0（与 Core1 的 LHDC 解码分核），
 * 优先级高于 Core0 其余应用任务，保证 DMA 不欠载。 */
#define I2S_TASK_STACK_SIZE       8192
#define I2S_TASK_PRIORITY         17
#define I2S_TASK_CORE             0

/* ============ 蓝牙 ============ */
/* BtAppTask：蓝牙应用事件队列（A2DP/AVRCP 事件分发）。Core1。
 * 优先级 = configMAX_PRIORITIES-10 = 15，低于同核 19/20/22 的协议栈任务。 */
#define BT_APP_STACK_SIZE         4096
#define BT_APP_PRIORITY           (configMAX_PRIORITIES - 10)
#define BT_APP_CORE               1

/* BtAvBgTask：蓝牙后台工作（I2S 重配/停止、存 BDA、自动回连）。Core0，
 * 不与 Core1 的 LHDC 解码器竞争。 */
#define BT_AV_BG_STACK_SIZE       4096
#define BT_AV_BG_PRIORITY         3
#define BT_AV_BG_CORE             0

/* bt_reconnect：A2DP 自动回连专用任务（不阻塞 BtAvBgTask 队列）。Core0。 */
#define BT_RECONNECT_STACK_SIZE   3072
#define BT_RECONNECT_PRIORITY     5
#define BT_RECONNECT_CORE         0

#endif /* __TASK_CONFIG_H */
