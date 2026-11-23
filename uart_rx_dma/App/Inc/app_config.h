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
 * 当前 CubeMX 生成代码（Core/Src/usart.c）里是 115200。
 * 改波特率请在 CubeMX 的 USART1 里改并重新生成；这里只作工程记录。 */
#ifndef APP_UART_BAUD
#define APP_UART_BAUD            115200U
#endif
#define APP_UART_INSTANCE        USART1

/* -------------------------------------------------------------------------- */
/* DMA / ring                                                                 */
/* -------------------------------------------------------------------------- */
/* DMA 循环缓冲：IDLE 事件按帧搬运，连续流由 HT/TC 每 512 B 兜底。
 * 921600 8N1 = 92160 B/s，1024 B = 11.1 ms；115200 时 88.9 ms。 */
#define APP_RX_DMA_BUF_SIZE      1024U

/* 软件环形缓冲：DMA 事件把新字节搬进来，主循环慢慢消费。
 * 8192 B = 88.9 ms 的 921600 满速流量（115200 时 711 ms）；
 * 1000 帧连续突发的净积压约 2 KB，放得下。必须为 2 的幂。 */
#define APP_RX_RING_SIZE         8192U

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

/* 帧内字节间超时：超过该时间没有新字节，判定为残帧并回 NACK。
 * 115200 下一帧 16 B 约 1.39 ms；50 ms 给 USB-TTL/驱动留足突发间隔，
 * 又能在人眼可感的时间内报出残帧。 */
#define APP_FRAME_TIMEOUT_MS     50U

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
