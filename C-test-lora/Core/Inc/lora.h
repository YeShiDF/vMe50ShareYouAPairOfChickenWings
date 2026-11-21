/**
 * @file    lora.h
 * @brief   DX-LR32-900T22D LoRa 模块驱动接口
 *
 * ===========================================================================
 * 一、硬件连接
 * ---------------------------------------------------------------------------
 *   模块引脚        接到哪儿              说明
 *   ------------    ------------------    --------------------------------
 *   3  RX            MCU USARTx_TX         串口是交叉连接，别接成 TX-TX
 *   4  TX            MCU USARTx_RX
 *   5  AUX           暂不接                保留给发送节流用（宏已定义未使用）
 *   6  VCC           3.3V / 5V             就近并 100uF + 0.1uF，发射瞬间约 190mA
 *   7  GND           MCU GND               必须共地，否则所有信号都是浮空
 *   1/2 M0 M1       不接                  AT 模式下这两个是模块的输出脚
 *
 *   车端  ：USART2  PA2(TX) / PA3(RX)，9600 8N1，DMA1_Channel6 接收
 *   遥控端：USART1  PA9(TX) / PA10(RX)，中断接收
 *   注意：本工程 usart.c 里 USART1 是 115200、USART2 是 9600，而模块出厂默认 9600。
 *         如果遥控端走 USART1，务必确认波特率和模块一致，否则 AT 全部无响应。
 *
 * ===========================================================================
 * 二、通信模式：定点传输（AT+MODE1）
 * ---------------------------------------------------------------------------
 *   发送时数据必须先加 3 字节头：目标地址高字节 + 目标地址低字节 + 目标信道。
 *   接收端模块会自动剥掉这 3 字节，MCU 收到的就是纯业务数据。
 *
 *       发送方向：  [MAC_H][MAC_L][CH][payload ...]   （由 lora_send 自动拼）
 *       接收方向：  [payload ...]                     （模块已剥头）
 *
 *   两端必须满足：LEVEL 相同、CHANNEL 相同、目标地址写对，否则收不到且不会报错。
 *
 * ===========================================================================
 * 三、编译开关
 * ---------------------------------------------------------------------------
 *   IFCTRL      选择本机角色，直接决定 MAC、串口、接收方式，见 lora.c
 *   LORA_DEBUG  置 1 后把错误码存进 errList，方便调试时回看历史
 *
 * ===========================================================================
 * 四、使用前提
 * ---------------------------------------------------------------------------
 *   调用 lora_init() 之前必须完成 USART 初始化；
 *   接收依赖"中断"或"DMA"至少开启一项，否则只能发不能收。
 * ===========================================================================
 */

#ifndef C_TEST_LORA_LORA_H
#define C_TEST_LORA_LORA_H

/* ---------------------------- 网络参数 ---------------------------------- */
/* 两台设备的地址，写进模块内部；定点模式下发送方要用对方的地址做包头。
 * 注意：这里是"本机地址"，不是"目标地址"，目标地址由 lora_client 决定 */
#define CAR_MAC         0x9527      /* 车端地址 */
#define CTRL_MAC        0x0721      /* 遥控器地址 */

#define LORA_CHANNEL    0x10        /* 信道，00~63（十六进制），两端必须一致 */
#define LORA_MODE       1           /* 传输模式：0=透传 1=定点 2=广播 */
#define LORA_LEVEL      4           /* 空中速率等级，4 约 10417bps；两端必须一致 */

/* ---------------------------- 编译开关 ---------------------------------- */
/* 本机角色：0 = 车端，1 = 遥控器端
 *   IFCTRL = 0 → 车端    ：CAR_MAC ，USART2 + DMA1_Channel6 接收，USART1 当调试口
 *   IFCTRL = 1 → 遥控器端：CTRL_MAC，USART1 + 中断接收
 * lora.c 里用的是 #if !IFCTRL，所以 0 走第一个分支，改开关时记得两边注释一起看 */
#define IFCTRL          0

/* AUX 引脚：模块发送/接收期间为高电平。当前代码没有用它做流控，仅保留定义 */
#define LORA_AUX_PIN    GPIO_PIN_8
#define LORA_AUX_PORT   GPIOA

/* 调试开关：1=记录错误码到 errList；0=errReport 编译成空函数，无额外开销 */
#define LORA_DEBUG      0

#include <stdint.h>

#include "cmsis_os2.h"
#include "main.h"

/* ---------------------------- 全局变量 ---------------------------------- */
extern uint8_t rxBit;                       /* 接收中转：每收到 1 字节先落在这里 */
extern osMessageQueueId_t cmdQueueHandle;   /* 收到的字节逐个数进这个队列 */

/**
 * @brief LoRa 模块实例
 *
 * 一个实例描述"一块模块 + 它挂在哪个串口上"。目前只用一个（lora_local），
 * 另一个（lora_client）只是个地址容器，用来拼发送包头。
 */
typedef struct {
    uint16_t mac;               /* 本机地址，写进模块 */
    uint8_t  channel;           /* 本机信道，写进模块 */

    GPIO_TypeDef *rx_port;      /* 预留：接模块 RX 的 MCU 引脚端口 */
    GPIO_TypeDef *tx_port;      /* 预留：接模块 TX 的 MCU 引脚端口 */
    uint16_t      rx_pin;       /* 预留：引脚号 */
    uint16_t      tx_pin;       /* 预留：引脚号 */

    UART_HandleTypeDef *huart;  /* 模块挂载的串口句柄 */
    DMA_HandleTypeDef  *hdma_rx;/* 接收 DMA；填 NULL 表示改用中断接收 */
    DMA_HandleTypeDef  *hdma_tx;/* 发送 DMA；当前未使用（发送走阻塞方式） */
} lora_t;

/**
 * @brief 驱动返回码
 */
typedef enum {
    normal = 0,     /* 成功 */
    error,          /* 通用错误（预留给上层） */
    config_null,    /* 参数非法：空指针或长度为 0 */
    send_time_out,  /* 预留：发送超时（当前未使用） */
    send_error,     /* 串口发送失败 */
    rx_error,       /* 挂载接收失败（串口忙 / DMA 配置缺失） */
} lora_state_t;

/* ---------------------------- 对外接口 ---------------------------------- */
extern lora_t lora_client, lora_local;

/**
 * @brief  初始化模块：进 AT → 逐条配置 → 复位 → 挂接收
 * @retval normal 流程执行完毕
 * @note   内部全是阻塞等待，整个过程约 1.5 秒；
 *         当前不校验模块应答，模块没接也会返回 normal（靠 pc13 灯或串口日志判断）
 */
lora_state_t lora_init(void);

/**
 * @brief  挂载一次接收（有 DMA 用 DMA，没有就用中断）
 * @retval normal 挂载成功；rx_error 启动失败
 * @note   每收到 1 字节后必须重新调用，否则只能收到 1 个字节
 */
lora_state_t lora_receive(void);

/**
 * @brief  发送一帧数据，自动补 3 字节定点地址头
 * @param  data 负载首地址，不能为 NULL
 * @param  size 负载长度，调用方需保证 <= 64（内部缓冲只有 3+64 字节）
 * @retval normal 发送完成；config_null 参数非法；send_error 串口发送失败
 * @note   阻塞发送，9600bps 下发 10 字节约 13ms，不要在中断里调用
 */
lora_state_t lora_send(uint8_t *data, uint8_t size);

/**
 * @brief  通过调试串口输出字符串（仅车端有效，遥控端是空实现）
 */
void dbg_send(const char *s);

/**
 * @brief  记录一个错误码（LORA_DEBUG=0 时为空函数）
 */
void errReport(lora_state_t err);

#endif /* C_TEST_LORA_LORA_H */
