/**
 * @file    app_config.h
 * @brief   工程级可调参数：串口、DMA、环形缓冲、帧格式、超时。
 *
 * 所有"魔法数字"集中在这里，方便按板子/波特率重新算余量。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* UART                                                                       */
/* -------------------------------------------------------------------------- */
/* 硬件：USART1 = PA9(TX) / PA10(RX)，AF7，CH343 USB-TTL 接这里。
 * 板子原设计 USART1 跑 921600；如果你的 USB-TTL / CubeMX 是 115200，
 * 编译时加 -DAPP_UART_BAUD=115200 即可（见 README）。 */
#ifndef APP_UART_BAUD
#define APP_UART_BAUD            921600U
#endif
#define APP_UART_INSTANCE        USART1

/* -------------------------------------------------------------------------- */
/* DMA / ring                                                                 */
/* -------------------------------------------------------------------------- */
/* DMA 循环缓冲：DMA 每写满一半（512 B）触发一次 HT/TC 中断。
 * 115200 8N1 = 11520 B/s，512 B = 44.4 ms，即中断延迟 < 44 ms 就不会被覆盖。 */
#define APP_RX_DMA_BUF_SIZE      1024U

/* 软件环形缓冲：DMA 中断把半个 DMA 缓冲搬进来，主循环慢慢消费。
 * 4096 B = 355 ms 的 115200 流量；必须为 2 的幂。 */
#define APP_RX_RING_SIZE         4096U

/* 主循环一次最多从环形缓冲取多少字节。取小一点可以让解析/回包更均匀。 */
#define APP_POLL_CHUNK           64U

/* -------------------------------------------------------------------------- */
/* 帧格式                                                                     */
/* -------------------------------------------------------------------------- */
/* | SOF 0xA5 | TYPE | LEN | PAYLOAD[LEN] | CRC32(4, LE) | EOF 0x5A |
 *   CRC32 覆盖 TYPE + LEN + PAYLOAD，不覆盖 SOF/EOF。
 *   LEN = 0..8，整帧 = 8 + LEN <= 16 字节。 */
#define APP_FRAME_SOF            0xA5U
#define APP_FRAME_EOF            0x5AU
#define APP_FRAME_MAX_PAYLOAD    8U
#define APP_FRAME_MAX_SIZE       (APP_FRAME_MAX_PAYLOAD + 8U)

/* 帧内字节间超时：超过该时间没有新字节，判定为残帧并回 NACK。 */
#define APP_FRAME_TIMEOUT_MS     20U

/* -------------------------------------------------------------------------- */
/* 版本                                                                       */
/* -------------------------------------------------------------------------- */
#define APP_FW_VERSION           0x0001U

#if (APP_RX_RING_SIZE & (APP_RX_RING_SIZE - 1U)) != 0U
#error "APP_RX_RING_SIZE must be a power of two"
#endif

#if (APP_RX_DMA_BUF_SIZE & 1U) != 0U
#error "APP_RX_DMA_BUF_SIZE must be even"
#endif

#endif /* APP_CONFIG_H */
