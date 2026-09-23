/**
  ******************************************************************************
  * @file    ota_trace.h
  * @brief   极早期启动诊断：不依赖 HAL 初始化顺序 / FreeRTOS，直接怼 USART1 打字符
  *
  * 用途：从 Bootloader 跳过来之后 App 一片安静时，定位到底卡在哪一步。
  * 在 `main()` 的最开头先 `OtaTrace_Init()`，之后每条关键初始化后面打一个标记：
  * 串口上最后看到哪个标记，就说明它卡在下一个语句里。
  *
  * ⚠ 只在 **Debug 构建**（`-DDEBUG`）里真的输出；Release 里全部是空函数，
  *   一行代码都不会多，也不占 Flash（`-Wl,--gc-sections` 会把它们扔掉）。
  *
  * 它自己会把 USART1 的 RCC/GPIO/波特率配好，所以哪怕 App 后面的 UART 初始化
  * 失败了也能打出来 —— 这正是要排查的情况。
  ******************************************************************************
  */

#ifndef __OTA_TRACE_H__
#define __OTA_TRACE_H__

#include <stdint.h>

/* 配好 USART1（PA9/PA10）用于打诊断字符。可以在 main() 第一行就调用 */
void OtaTrace_Init(void);

/* 打一段文本（Release 下什么都不做） */
void OtaTrace_Text(const char *text);

/* 打一个字符（Release 下什么都不做） */
void OtaTrace_Mark(char mark);

/*
 * 故障现场：在 HardFault / MemManage / BusFault / UsageFault 的 handler 里调 ——
 * **必须用下面的 OTATRACE_FAULT() 宏，不要直接调这个函数**。
 *
 * 为什么要传 frame_sp/exc_lr：异常帧压在**哪条栈**上（任务用 PSP / handler 用 MSP）
 * 只能从 EXC_RETURN(LR) 看出来，而进函数后 LR 随时会被编译器用掉；
 * 而且我们自己的函数也会动 MSP，直接 `mrs msp` 得到的地址和异常帧对不上。
 * 以前的版本就是无条件 `mrs msp` → 任务里出错时打出来的 R0-R3/PC/LR 全是主栈上的垃圾值
 * （2026-09-19 拿那个假 PC 反查了半天符号，白费一轮）。
 *
 * 输出用**裸寄存器**（不依赖 HAL/tick/互斥量），故障现场只有这个靠得住；
 * 内部全是有界循环，不会再卡死。只在 Debug 构建里真输出。
 */
void OtaTrace_Fault(const char *tag, uint32_t frame_sp, uint32_t exc_lr);

/*
 * 在异常 handler 的 USER CODE 区里这样用（必须是 handler 里的第一句）：
 *
 *     void HardFault_Handler(void)
 *     {
 *       OTATRACE_FAULT("HardFault");
 *       while (1) { }
 *     }
 *
 * 宏自己从 LR 取 EXC_RETURN 并据此选栈：tst lr,#4 → 0 用 MSP，1 用 PSP。
 */
#define OTATRACE_FAULT(tag)                                              \
    do                                                                   \
    {                                                                    \
        uint32_t ota_trace_exc_lr_;                                      \
        uint32_t ota_trace_frame_sp_;                                    \
        __asm volatile ("mov %0, lr" : "=r" (ota_trace_exc_lr_));        \
        __asm volatile ("tst lr, #4 \n\t"                                \
                        "ite eq \n\t"                                   \
                        "mrseq %0, msp \n\t"                            \
                        "mrsne %0, psp"                                  \
                        : "=r" (ota_trace_frame_sp_));                   \
        OtaTrace_Fault((tag), ota_trace_frame_sp_, ota_trace_exc_lr_);   \
    } while (0)

/* FreeRTOS 的 configASSERT 失败时调：把表达式 + 文件名 + 行号打出来 */
void OtaTrace_AssertFailed(const char *file, int line, const char *expr);

#endif /* __OTA_TRACE_H__ */
