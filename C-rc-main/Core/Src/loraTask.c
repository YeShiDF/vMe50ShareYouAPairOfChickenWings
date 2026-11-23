/**
 * @file    loraTask.c
 * @brief   LoRa 业务任务：从队列取字节，攒成指令再执行
 *
 * ===========================================================================
 * 它和中断回调的分工
 * ---------------------------------------------------------------------------
 *   it_callback.c ：只管"把字节收进来"，一字节一个消息，绝不解析
 *   loraTask.c    ：只管"把字节拼成指令"，解析、校验、执行都在这里
 *
 * 队列 cmdQueue 的元素是 uint8_t，所以一条多字节指令要自己攒。
 * 两种常见做法：
 *   a) 定长协议：每条指令固定 N 字节，收满 N 个就处理（最简单）
 *   b) 带帧头：0xAA + 长度 + 数据 + 校验和，按帧头对齐（能自恢复）
 * ===========================================================================
 */

#include <stdlib.h>

#include "cmsis_os2.h"
#include "lora.h"
#include "freertos_os2.h"
#include "task.h"
#include "usart.h"

/**
 * @brief 指令处理任务
 * @param argument 任务参数（CubeMX 创建时传 NULL，这里不用）
 *
 * @note 任务函数不能返回。一旦 return，FreeRTOS 会进 prvTaskExitError()
 *       关中断死循环，整块板子就停摆了。
 */
void StartTaskMainCmd(void *argument) {
    (void)argument;

    uint8_t cmd = 0x00;

    for (;;) {
        cmd = 0x00;

        /* 队列里没数据时在这里阻塞，不占 CPU；来一个字节就唤醒一次 */
        osMessageQueueGet(cmdQueueHandle, &cmd, 0, osWaitForever);

        /* --------------------------------------------------------------
         * 业务解析区
         * 当前只把字节取出来，还没做拼帧。典型写法：
         *
         *   static uint8_t frame[16];
         *   static uint8_t idx = 0;
         *   frame[idx++] = cmd;
         *   if (idx >= 4) {            // 假设定长 4 字节
         *       HandleCommand(frame);  // 解析并执行
         *       idx = 0;
         *   }
         *
         * 注意：这里不要做耗时操作，会把后面到达的字节堵在队列里。
         * -------------------------------------------------------------- */
    }
}
