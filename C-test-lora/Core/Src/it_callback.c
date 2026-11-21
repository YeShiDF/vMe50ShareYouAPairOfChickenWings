/**
 * @file    it_callback.c
 * @brief   HAL 中断回调集中存放处
 *
 * ===========================================================================
 * 接收链路（每收到 1 字节走一遍）
 * ---------------------------------------------------------------------------
 *   串口收到字节
 *     → HAL_UART_IRQHandler（DMA 模式下由 DMA 传输完成中断代替）
 *       → HAL_UART_RxCpltCallback ← 就是下面这个函数
 *         → 把 rxBit 投进 cmdQueue（深度 16）
 *         → 立刻重新挂接收（DMA 或中断，由 lora_receive 自动选）
 *
 * ===========================================================================
 * 写这个文件时的三条纪律
 * ---------------------------------------------------------------------------
 *   1. 这里跑在中断上下文，只能调用 "FromISR 安全" 的 API。
 *      CMSIS-RTOS2 的 osMessageQueuePut 内部会判断是否在中断里，可以放心用；
 *      但 HAL_Delay / 阻塞版 HAL_UART_Transmit 绝对不能碰。
 *
 *   2. 中断优先级必须 >= 5（configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY）。
 *      给 0~4 的话，第一次进回调就会踩 FreeRTOS 的 configASSERT，直接死机。
 *
 *   3. "收到 1 字节"只发生一次。回调里不重新挂接收，就永远只收到那 1 个字节。
 * ===========================================================================
 */

#include "main.h"
#include "lora.h"

/**
 * @brief 串口接收完成回调（DMA 模式和中断模式共用这一个）
 * @param huart 触发回调的串口句柄
 *
 * @note 队列深度只有 16。如果上层任务处理太慢，osMessageQueuePut 超时写 0
 *       会直接丢字节——这是预期行为，丢了不会有任何提示。
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    /* 只处理本机 LoRa 挂的那个串口，其它串口的数据一概不碰 */
    if (huart == lora_local.huart) {
        /* 1. 把刚收到的 1 字节交给业务任务（不等待，队列满就丢） */
        osMessageQueuePut(cmdQueueHandle, &rxBit, 0, 0);

        /* 2. 重新武装接收，否则下一字节不会产生回调 */
        lora_receive();
    }
}
