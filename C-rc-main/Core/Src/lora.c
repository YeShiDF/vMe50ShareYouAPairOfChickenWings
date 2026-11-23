/**
 * @file    lora.c
 * @brief   LoRa 模块驱动实现：AT 初始化、发送打包、接收挂载
 *
 * ===========================================================================
 * 数据流总览
 * ---------------------------------------------------------------------------
 *   发送：
 *     lora_send()
 *        → 拼 3 字节包头 [MAC_H][MAC_L][CH]
 *        → HAL_UART_Transmit 阻塞发出去
 *        → 模块按包头找到目标，转成空口数据
 *
 *   接收：
 *     模块收到空口数据 → 从 TX 脚吐字节
 *        → USART 每收 1 字节触发 HAL_UART_RxCpltCallback（见 it_callback.c）
 *        → 字节塞进 cmdQueue
 *        → 回调里再挂一次接收
 *        → StartTaskMainCmd() 从队列逐字节取出，攒成指令
 *
 * ===========================================================================
 * 为什么一次只收 1 字节//备注：其实是因为一字节做指令长度完全够了，还简单
 * ---------------------------------------------------------------------------
 *   定点模式下模块靠"串口空闲"判断一包结束，逐字节收不需要预知长度，配合
 *   队列最省事。代价是每字节一次中断（9600bps 下约 960 次/秒，M3 完全扛得住）。
 *   以后数据量变大，建议改成"定长 DMA + IDLE 中断"，一次搬一整包。
 * ===========================================================================
 */

#include "lora.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os2.h"
#include "usart.h"
#include "main.h"

/* ===========================================================================
 * 一、本机 / 对端配置
 * ---------------------------------------------------------------------------
 * lora_local  ：本机模块，决定用哪个串口、哪个 MAC、怎么收
 * lora_client ：对端地址，只用来拼发送包头（不占用串口）
 *
 * 编译开关 IFCTRL 来自 lora.h：
 *   IFCTRL = 0 → 车端（走下面第一个分支）：CAR_MAC，USART2 + DMA 接收
 *   IFCTRL = 1 → 遥控器端（走 #else）：CTRL_MAC，USART1 + 中断接收
 * 注意条件写的是 !IFCTRL，因为 0 表示车端，编译时取反才会走车端分支。
 *
 * 换板子时只改这一段，其余代码不用动。
 * ===========================================================================*/

#if !IFCTRL
/* ------------------------------ 车端 ------------------------------------ */
lora_t lora_local = {
    .mac     = CAR_MAC,             /* 本机地址：车 */
    .channel = LORA_CHANNEL,
    .rx_port = GPIOA,
    .tx_port = GPIOA,
    .rx_pin  = 2,                   /* PA2 = USART2_TX，接模块 RX */
    .tx_pin  = 3,                   /* PA3 = USART2_RX，接模块 TX */
    .huart   = &huart2,
    .hdma_rx = &hdma_usart2_rx,     /* 车端用 DMA 收，见 usart.c 的 MspInit */
    .hdma_tx = &hdma_usart2_tx,
};

lora_t lora_client = {
    .mac     = CTRL_MAC,            /* 发送目标：遥控器 */
    .channel = LORA_CHANNEL,
};

/**
 * @brief 车端调试输出，走 USART1（板上 CH340 / USB-TTL）
 * @note  只有车端有这个口，所以放在 !IFCTRL（车端）分支里
 */
void dbg_send(const char *s) {
    HAL_UART_Transmit(&huart1, (const uint8_t *)s, (uint16_t)strlen(s), 200);
}

#else
/* ---------------------------- 遥控器端 ---------------------------------- */
lora_t lora_local = {
    .mac     = CTRL_MAC,            /* 本机地址：遥控器 */
    .channel = LORA_CHANNEL,
    .rx_port = GPIOA,
    .tx_port = GPIOA,
    .rx_pin  = 9,                   /* PA9  = USART1_TX，接模块 RX */
    .tx_pin  = 10,                  /* PA10 = USART1_RX，接模块 TX */
    .huart   = &huart1,
    .hdma_rx = NULL,                /* 遥控端没配 DMA，自动退回中断接收 */
    .hdma_tx = NULL,
};

lora_t lora_client = {
    .mac     = CAR_MAC,             /* 发送目标：车 */
    .channel = LORA_CHANNEL,
};

/**
 * @brief 遥控端没有独立调试口，调试输出直接丢弃
 * @note  这里写成空函数而不是宏，是为了和头文件里的声明保持一致。
 *        用宏的话只在本文件有效，别的文件调用就会链接失败。
 */
void dbg_send(const char *s) {
    (void)s;
}

#endif

/* 接收中转：DMA / 中断每次只往这里写 1 字节，回调立刻把它搬进队列 */
uint8_t rxBit = 0x00;

/* ===========================================================================
 * 二、错误记录（仅调试时启用）
 * ---------------------------------------------------------------------------
 * 打开 LORA_DEBUG 后，错误码会顺序存进 errList。跑一段时间再暂停调试器，
 * 就能看到"历史上出过哪些错"，不用打断点破坏时序。
 * ===========================================================================*/
#if LORA_DEBUG
lora_state_t errList[64] = {0};

void errReport(lora_state_t err) {
    static int index = 0;

    errList[index++] = err;
    /* 写满一轮就清空重来；本函数不会被传入 normal(0)，所以用 0 当"空位"判断 */
    if (errList[63] != 0) {
        memset(errList, 0, sizeof(errList));
        index = 0;
    }
}
#else
void errReport(lora_state_t err) {
    (void)err;      /* 不调试时什么也不做，编译后没有额外开销 */
}
#endif

/* ===========================================================================
 * 三、初始化
 * ===========================================================================*/

/**
 * @brief 初始化模块：进 AT → 配置参数 → 复位生效 → 挂接收
 *
 * 顺序不能乱：
 *   1. +++         进入 AT 模式（换码序列，不是普通 AT 命令）
 *   2. AT+LEVEL    空中速率等级，两端必须一致
 *   3. AT+MAC      本机地址，十六进制、逗号分隔
 *   4. AT+CHANNEL  信道，十六进制两位数，端点必须一致
 *   5. AT+MODE1    定点传输（发送时要带 3 字节地址头）
 *   6. AT+RESET    重启，参数才真正生效（手册明确要求）
 *   7. 挂接收
 *
 * @note 全程阻塞，约 1.5 秒；建议在 main 里、调度器启动前调用一次。
 *       当前不校验应答，模块没接也会走到 normal，调试时看 dbg_send 的输出。
 */
lora_state_t lora_init(void) {
    /* 上电后给模块一点启动时间，太早发 +++ 会被当成业务数据发到空中 */
    delay(500);

    char     msg[64] = {0};
    uint16_t size    = 0;

    /* --- 1. 尝试进入 AT 模式 -------------------------------------------- */
    HAL_UART_Transmit(lora_local.huart, "+++\r\n", 5, 100);
    HAL_UARTEx_ReceiveToIdle(lora_local.huart, msg, 64, &size, 1000);

    delay(50);
    /* 没收到 "Entry" 就再补一次。
     * 注意：+++ 是"切换"语义，第二次发有可能把模块踢回透传模式，
     *       所以这里只补一次，且后面各步不依赖它的成败。 */
    if (strstr(msg, "Entry") == NULL) {
        HAL_UART_Transmit(lora_local.huart, "+++\r\n", 5, 100);
    }
    delay(50);

    /* --- 2. 速率等级 ---------------------------------------------------- */
    memset(msg, 0, sizeof(msg));
    char b[20] = {0};
    sprintf(b, "AT+LEVEL%d\r\n", LORA_LEVEL);
    HAL_UART_Transmit(lora_local.huart, b, strlen(b), 100);
    HAL_UARTEx_ReceiveToIdle(lora_local.huart, msg, 64, &size, 1000);
    dbg_send(msg);                      /* 车端能看到模块回的 OK / EEROR */

    /* --- 3. 本机地址（十六进制，高字节在前） ---------------------------- */
    delay(50);
    memset(msg, 0, sizeof(msg));
    memset(b, 0, sizeof(b));
    sprintf(b, "AT+MAC%02x,%02x\r\n", (uint8_t)(lora_local.mac >> 8),
                                      (uint8_t)(lora_local.mac));
    HAL_UART_Transmit(lora_local.huart, b, strlen(b), 100);
    HAL_UARTEx_ReceiveToIdle(lora_local.huart, msg, 64, &size, 1000);
    dbg_send(msg);

    /* --- 4. 信道（十六进制两位数，范围 00~63） -------------------------- */
    delay(50);
    memset(msg, 0, sizeof(msg));
    memset(b, 0, sizeof(b));
    sprintf(b, "AT+CHANNEL%02x\r\n", lora_local.channel);
    HAL_UART_Transmit(lora_local.huart, b, strlen(b), 100);
    HAL_UARTEx_ReceiveToIdle(lora_local.huart, msg, 64, &size, 1000);
    dbg_send(msg);

    /* --- 5. 定点传输模式 ------------------------------------------------
     * 改 LORA_MODE 时记得同步这里的字符串，当前写死为 1 */
    delay(50);
    memset(msg, 0, sizeof(msg));
    HAL_UART_Transmit(lora_local.huart, "AT+MODE1\r\n", 10, 100);
    HAL_UARTEx_ReceiveToIdle(lora_local.huart, msg, 64, &size, 1000);
    dbg_send(msg);

    /* --- 6. 复位，让前面的参数生效 -------------------------------------- */
    delay(50);
    memset(msg, 0, sizeof(msg));
    HAL_UART_Transmit(lora_local.huart, "AT+RESET\r\n", 10, 100);
    HAL_UARTEx_ReceiveToIdle(lora_local.huart, msg, 64, &size, 1000);
    dbg_send(msg);

    /* --- 7. 等模块重启完再挂接收 ----------------------------------------
     * 复位后模块会吐启动横幅，挂太早会把横幅当成用户数据 */
    delay(100);
    lora_receive();

    /* 发送格式备忘：地址高字节 + 地址低字节 + 信道 + 数据（由 lora_send 拼） */
    return normal;
}

/* ===========================================================================
 * 四、接收
 * ===========================================================================*/

/**
 * @brief 挂载一次接收，收满 1 字节后触发 HAL_UART_RxCpltCallback
 * @retval normal 挂载成功；rx_error 启动失败
 *
 * 按硬件配置自动二选一：
 *   有 hdma_rx → DMA 搬 1 字节，CPU 不参与
 *   没有 DMA   → 中断接收（要求该 USART 的 NVIC 已使能）
 */
lora_state_t lora_receive(void) {
    if (lora_local.hdma_rx == NULL) {
        /* 中断方式：CubeMX 里必须打开该 USART 的全局中断，否则永远收不到 */
        if (HAL_UART_Receive_IT(lora_local.huart, &rxBit, 1) != HAL_OK) {
            errReport(rx_error);
            return rx_error;
        }
        return normal;
    }

    /* DMA 方式：DMA1_Channel6 把 DR 里的字节直接搬到 rxBit */
    if (HAL_UART_Receive_DMA(lora_local.huart, &rxBit, 1) != HAL_OK) {
        errReport(rx_error);
        return rx_error;
    }
    return normal;
}

/* ===========================================================================
 * 五、发送
 * ===========================================================================*/

/**
 * @brief 发送一帧数据，自动在前面补 3 字节定点地址头
 * @param data 负载首地址，不能为 NULL
 * @param size 负载长度，调用方需保证 <= 64（内部缓冲 = 3 + 64）
 * @retval normal 发送完成；config_null 参数非法；send_error 串口发送失败
 *
 * 为什么拼成一个 buffer 再发：
 *   定点模式下模块靠串口空闲间隔分包。如果包头和数据分两次 Transmit，
 *   中间被打断超过几毫秒，模块会把后面的数据当成新包，前 3 字节被当地址吃掉。
 *   一次发完就没有这个缝隙。
 */
lora_state_t lora_send(uint8_t *data, uint8_t size) {
    if (data == NULL || size == 0) return config_null;

    uint8_t buf[3 + 64] = {0};
    memcpy(buf + 3, data, size);

    /* 目标地址：高字节 + 低字节 + 目标信道 */
    buf[0] = (uint8_t)(lora_client.mac >> 8);
    buf[1] = (uint8_t)(lora_client.mac);
    buf[2] = (uint8_t)(lora_client.channel);

    if (HAL_UART_Transmit(lora_local.huart, buf, (uint16_t)(3 + size), 100) != HAL_OK) {
        errReport(send_error);
        return send_error;
    }
    return normal;
}

/* ===========================================================================
 * 六、待办 / 已知限制
 * ---------------------------------------------------------------------------
 * 1. 收包是逐字节进队列，没有帧头、长度、校验，上层只能按固定长度解析。
 *    数据变复杂后建议加：0xAA 包头 + 长度 + 数据 + 校验和。
 * 2. 发送没有做速率限制。LEVEL4 下一包 10 字节约 9ms，理论可到 100Hz；
 *    LEVEL2 下要约 40ms，发太快会在模块内部堆积丢包，那时需要接 AUX 做流控。
 * 3. lora_init 不校验应答，"配置成功"和"模块根本没接"返回值一样。
 * ===========================================================================*/
