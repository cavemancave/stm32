/**
  ******************************************************************************
  * @file    motor_ctrl.c
  * @brief   电机业务流程：无线命令驱动的速度环 / 位置环控制，外加状态轮询
  *
  * 两条控制路径（都用无线命令触发，没有按键流程）：
  *   - `ctrl speedloop/speed/accel/drive` = **速度环（0x02）**，给两轮差速小车用：
  *     "一直以某个转速转"。为什么轮子不用位置环，见下面「速度环」那一节的注释。
  *   - `ctrl posloop/movepos/movedeg` = **位置环（0x03）**，"转到位就停"，
  *     给机械臂/云台那种关节用。
  *
  * 这一层只关心「业务」，不碰串口：
  *   所有 Motor_xxx() 调用（Motor_Enable / Motor_QueryStatus / Motor_SetValue ...）
  *   内部都会走 MotorIo_Exchange()，由收发任务串行执行，所以这里可以随便调。
  *
  * 轮询为什么要单独一个 task：
  *   osTimer 的回调跑在 timer service task 里，一旦在里面阻塞（等串口 / 等队列），
  *   所有软件定时器都会被卡住。所以回调只干一件事：释放信号量，
  *   真正的收发放到 MotorCtrl_PollTask 里做。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_ctrl.h"

#include "motor.h"
#include "motor_fmt.h"
#include "motor_drive.h"    /* 小车底盘驱动：里程差闭环 + 看门狗，跑在它自己的任务里 */
#include "motor_power.h"    /* 电机电源开关（PC14） */
#include "uart_log.h"

/* 无线控制命令的状态码（OTA_OK / OTA_E_xxx）和这个入口的约定在这里 */
#include "ota_layout.h"
#include "ota_trace.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include <stdio.h>

/* 轮询定时器（在 Core/Src/freertos.c 里创建），升级期间要把它停下来 */
extern osTimerId_t motorPosHandle;

/* Private define ------------------------------------------------------------*/

/* 挂在同一条串口上的电机台数（宏在 motor_ctrl.h：motor_drive.c 也要用它开数组）。
   现在 2 台：ID1 + ID2，共用同一条 USART10 单总线、也共用 PC14 那一路电机电源。
   ⚠ 达妙这颗的 ID 是**上电时锁存**的（ID 脚 低 = ID1 / 高 = ID2），所以一条总线最多两台。 */
/* 日志行缓冲大小（调用方栈上，别放在这个文件里当全局，省得抢）。
   256 是算出来的：最长的状态行（含 position 的 0.1°、故障码文本和 10 字节 raw）
   最坏情况约 168 字节，留足余量；改短了 gcc 会报 -Wformat-truncation。 */
#define MOTOR_LINE_SIZE             256U

/* 短行缓冲：只给“肯定短”（< 140 字节）且**处在深调用链里**的日志用。
   ⚠ 栈是按嵌套深度叠加的：MOTOR_LINE_SIZE 的行缓冲如果和另一个嵌套的函数同时活着，
     光两个行缓冲就要 700 字节栈。实测踩过：无线命令那条链被压深了 ~550 字节，
     otaSvc 任务的 2 KB 栈被压爆，而症状是 FreeRTOS 里一个毫不相干的断言
     （栈溢出把堆里挨着的队列控制块写坏了），查了很久。能省就省。 */
#define MOTOR_LINE_SHORT            144U

/* 10 字节 raw 打成文本的最长长度：2*10 + 9 个空格 + '\0' */
#define MOTOR_HEX_SIZE              32U

/* 0x64 的 DATA[6] 加速时间：速度环下每 1rpm 用多少 ms；位置环填 0（按 1 处理） */
#define MOTOR_MOVE_ACCEL_TIME       0x00U

/* 0x64 的 DATA[7] 刹车：0xFF 刹车，其它值不刹车（速度环下有效） */
#define MOTOR_MOVE_BRAKE            0x00U

/* ---- 速度环（两轮差速小车用这个环） ---- */

/* 速度环给定值的量纲（-3800~3800 ↔ -380~380rpm，1 单位 = 0.1rpm）在 motor_ctrl.h 里，
   因为无线命令那一层要用它做范围检查：MOTOR_SPEED_MAX_RAW */

/* 速度环下 0x64 的加速时间：每 1rpm 用多少 ms（0 按 1 处理，见手册）。
   上电默认给一个柔和的起步：0→30rpm 用 0.3s。想更跟手就发 `ctrl accel 0`。 */
#define MOTOR_SPEED_ACCEL_DEFAULT   10U

/* 切模式（使能之外，比如切速度环）失败时的重试：此刻电机肯定已经在应答了，
   丢一两帧重试两下就够（不像使能那样要等它上电） */
#define MOTOR_SETMODE_RETRY_MS      100U
#define MOTOR_SETMODE_RETRY_MAX     3U

/* ---- 使能重试 ---- */

/* 手册「操作步骤」：电机刚上电要过一会儿才应答，第一条命令（使能）必须反复重试 */
#define MOTOR_ENABLE_RETRY_MS       200U
#define MOTOR_ENABLE_RETRY_MAX      25U      /* 最多约 5s */

/* ---- 失能重试 ---- */

/* 失能时电机肯定已经在上电工作（刚刚回过状态帧 / 转动反馈），
   不像使能那样要等它上电，丢一帧重试两次就够 */
#define MOTOR_DISABLE_RETRY_MS      100U
#define MOTOR_DISABLE_RETRY_MAX     3U

/* 轮询任务栈（字节，CMSIS-RTOS2 的 stack_size 单位是字节）。
   里面要跑 snprintf（约 100-200 字节），再加上上面的行缓冲，留 2KB 保险 */
#define MOTOR_POLL_TASK_STACK       (512U * 4U)

/* Private variables ---------------------------------------------------------*/

/* 这条串口上挂的电机 ID，按顺序依次问（改这里的同时改 MOTOR_COUNT）
   顺序就是无线命令里的电机序号：1 = motor_ids[0]（总线 ID1）、2 = motor_ids[1]（ID2） */
static const uint8_t motor_ids[MOTOR_COUNT] = { 1U, 2U };

/* 无线命令里的"电机序号"（1 起）→ 总线 ID。序号越界返回 0（= 没有这台）。
   motor_drive.c 也用它把"第几台"换成帧首字节，所以是公开的。 */
uint8_t MotorCtrl_MotorIdOfIndex(uint8_t index)
{
    if ((index >= 1U) && (index <= MOTOR_COUNT))
    {
        return motor_ids[index - 1U];
    }

    return 0U;
}

static osSemaphoreId_t s_poll_sem  = NULL;
static osThreadId_t    s_poll_task = NULL;

static const osThreadAttr_t s_poll_task_attr =
{
    .name       = "motorPoll",
    .stack_size = MOTOR_POLL_TASK_STACK,
    .priority   = (osPriority_t)osPriorityBelowNormal,
};

/* 1 = 无线 OTA 升级中：电机强制失能、停轮询、禁止再命令走动。
   由 OtaService 在会话开始/结束时置位/清除（见 MotorCtrl_SetOtaMode） */
static uint8_t s_ota_mode = 0U;

/* 200 ms 轮询被暂停的次数（不是布尔值）：OTA 会话和别的模块各会停一次，
   计数归零才真的把定时器开回来 —— 用布尔值的话，先恢复的那一个
   会把另一个还需要的“暂停”也一并解开。 */
static uint8_t s_poll_pause_cnt = 0U;

/* 每台电机的"速度环加速时间"（每 1rpm 多少 ms，0x64 的 DATA[6]），下标 = motor_ids[] 下标。
   放在 RAM 里、上电默认 MOTOR_SPEED_ACCEL_DEFAULT；无线命令 ctrl accel 直接改它。
   用按 index 的数组而不是按总线 ID，是为了跟 motor_ids[] 的下标（= 命令里的"几号机"）对齐。 */
static uint8_t s_speed_accel[MOTOR_COUNT];

/* 「我们已经确认过这台在速度环」的缓存 —— 命中就不必每条命令都先查一次 0x75。

   ⚠ 这比"每次都查"省一趟总线往返（10 字节帧 @38400 一来一回 ~5ms + HAL/队列开销），
     在"两台背靠背给转速"那条路上尤其重要：0x75 要是夹在两条 0x64 中间，
     两台的起步差就从 ~10ms 放大成"一次查询那么长"。

   ⚠⚠ 所以**只有会让电机离开速度环的动作**才能清它，漏一处就会瞎发：
       失能（单台/全部）、电机电源断电或断电重启、进 OTA、切进位置环、
       以及 0x64 收不到合法 0x65（兜底：下次自动重新确认一次）。
     清掉的代价只是"下一条命令多一趟 0x75"，所以宁可多清。 */
static uint8_t s_in_speed_loop[MOTOR_COUNT];

/* Private function prototypes -----------------------------------------------*/
static void           MotorCtrl_PollTask(void *argument);
static void           MotorCtrl_PollOnce(void);
static void           MotorCtrl_HexToText(const uint8_t *data, uint8_t len,
                                          char *out, size_t out_size);
static uint8_t        MotorCtrl_SetModeRetry(uint8_t motor_id, uint8_t mode,
                                             MotorMode *ack, uint32_t retry_ms,
                                             uint32_t retry_max);
static void           MotorCtrl_ForgetSpeedLoop(uint8_t motor_id);
static uint8_t        MotorCtrl_Enable(uint8_t motor_id, MotorMode *ack);
static uint8_t        MotorCtrl_Disable(uint8_t motor_id, MotorMode *ack);
static void           MotorCtrl_DisableOne(uint8_t motor_id);
static void           MotorCtrl_DisableAll(void);
static void           MotorCtrl_Stop(uint8_t motor_id);
static uint8_t        MotorCtrl_EnterPositionLoop(uint8_t motor_id, MotorMode *ack);
static uint8_t        MotorCtrl_MoveToPosition(uint8_t motor_id, uint16_t position, const char *tag);

/* Private functions ---------------------------------------------------------*/

/* 10 字节 raw -> "01 75 00 ..."，和原来 main.c 里 %02X 手写的输出格式一样 */
static void MotorCtrl_HexToText(const uint8_t *data, uint8_t len, char *out, size_t out_size)
{
    size_t used = 0U;

    if ((out == NULL) || (out_size == 0U))
    {
        return;
    }

    out[0] = '\0';

    for (uint8_t i = 0U; i < len; i++)
    {
        int written = snprintf(&out[used], out_size - used,
                               (i == 0U) ? "%02X" : " %02X", data[i]);

        if (written <= 0)
        {
            break;
        }

        used += (size_t)written;

        if (used >= (out_size - 1U))
        {
            break;
        }
    }
}

/*
 * 发 0xA0 切模式（使能 / 失能），失败就隔 retry_ms 再来，最多 retry_max 次。
 *   反馈里 DATA[2] 是**切换之后的实际模式**，不是回显发过去的模式值 ——
 *   实测发 0x08 使能后回帧 01 A1 01 00 00 00 00 00 00 E0，模式值 0x01 = 默认的电流环。
 *   所以判成功只看「有没有收到合法的 0xA1 回帧」。
 */
static uint8_t MotorCtrl_SetModeRetry(uint8_t motor_id, uint8_t mode,
                                     MotorMode *ack, uint32_t retry_ms,
                                     uint32_t retry_max)
{
    for (uint32_t attempt = 0U; attempt < retry_max; attempt++)
    {
        if (Motor_SetMode(motor_id, mode, ack) != 0U)
        {
            return 1U;
        }

        osDelay(retry_ms);
    }

    return 0U;
}

/* 使能：电机刚上电要过一会儿才应答，必须反复重试（最多约 5s） */
static uint8_t MotorCtrl_Enable(uint8_t motor_id, MotorMode *ack)
{
    return MotorCtrl_SetModeRetry(motor_id, MOTOR_MODE_ENABLE, ack,
                                  MOTOR_ENABLE_RETRY_MS, MOTOR_ENABLE_RETRY_MAX);
}

/* 失能（0xA0/0x09）：此刻电机正在应答，重试几次就够 */
static uint8_t MotorCtrl_Disable(uint8_t motor_id, MotorMode *ack)
{
    return MotorCtrl_SetModeRetry(motor_id, MOTOR_MODE_DISABLE, ack,
                                  MOTOR_DISABLE_RETRY_MS, MOTOR_DISABLE_RETRY_MAX);
}

/* 急停：0x64 给定值 0（对应 cfg 里的「停止0RPM」） */
static void MotorCtrl_Stop(uint8_t motor_id)
{
    MotorValueAck ack;
    char          line[MOTOR_LINE_SIZE];
    char          hex[MOTOR_HEX_SIZE];

    uint8_t ok = Motor_SetValue(motor_id, 0, MOTOR_MOVE_ACCEL_TIME,
                                MOTOR_MOVE_BRAKE, &ack);

    MotorCtrl_HexToText(ack.raw, MOTOR_FRAME_SIZE, hex, sizeof(hex));

    (void)snprintf(line, sizeof(line),
                   "id=%u stop (0x64 value=0): reply=%s, speed=%d, current=%d, rx=%s\r\n",
                   (unsigned int)motor_id, (ok != 0U) ? "OK" : "NONE",
                   (int)ack.speed, (int)ack.current, hex);
    UartLog_Print(line);
}

/*
 * 切位置环：发 0xA0，模式值 MOTOR_MODE_POSITION(0x03)，回帧的模式值从 *ack 带出来。
 * ⚠ 手册 5.3 的模式表里**没有**位置环，0x03 来自厂家上位机工程
 *   （docs/M0603A 系列直驱电机.bit 的模式值下拉框里就有「位置环 = 03」）
 *   和 docs/M63A快捷指令集.cfg：「电机1切换位置环=1|0|16|01A003000000000000D9」。
 *
 * 返回 1 **只表示“收到了一条合法的 0xA1 回帧”，不代表电机真的进了位置环**：
 * 不认 0x03 的固件也会回一个合法的 0xA1，只是模式值不是 0x03。
 * 所以调用方（OTA_CTRL_POS_LOOP）在这之后都要再用 0x75/0x76 查一次模式复核。
 */
static uint8_t MotorCtrl_EnterPositionLoop(uint8_t motor_id, MotorMode *ack)
{
    char    line[MOTOR_LINE_SIZE];
    char    hex[MOTOR_HEX_SIZE];
    uint8_t mode_before;

    if (ack == NULL)
    {
        return 0U;
    }

    /* 切之前先看一眼当前模式（使能之后默认是 0x01 电流环），方便对比。
       ⚠ 这里的行缓冲开着的时候不要再嵌套一个带 256 字节行缓冲的函数 ——
         光那一对就要 700 字节栈，而本函数很可能是从无线命令（otaSvc 任务）
         这条路进来的（实测就把那个任务的栈压爆过，见 MOTOR_LINE_SHORT 的注释）。
         所以就地查一次模式值、用一行短的打出来。 */
    mode_before = MotorCtrl_QueryModeValue(motor_id);

    (void)snprintf(line, sizeof(line),
                   "id=%u mode before switch (expect 0x01 current loop): 0x%02X (%s)\r\n",
                   (unsigned int)motor_id, (unsigned int)mode_before,
                   Motor_ModeName(mode_before));
    UartLog_Print(line);

    uint8_t ok = Motor_SetMode(motor_id, MOTOR_MODE_POSITION, ack);

    /* 切走了就不在速度环了：把缓存清掉，下次给转速会先重新确认（多一趟 0x75） */
    MotorCtrl_ForgetSpeedLoop(motor_id);

    MotorCtrl_HexToText(ack->raw, MOTOR_FRAME_SIZE, hex, sizeof(hex));

    (void)snprintf(line, sizeof(line),
                   "id=%u set mode (0xA0/0x%02X position loop): reply=%s, mode=0x%02X (%s), rx=%s\r\n",
                   (unsigned int)motor_id, (unsigned int)MOTOR_MODE_POSITION,
                   (ok != 0U) ? "OK" : "NONE", (unsigned int)ack->mode,
                   Motor_ModeName(ack->mode), hex);
    UartLog_Print(line);

    return ok;
}

/*
 * 位置环下按**位置值**走：发 0x64（给定值 = 目标位置），等 0x65。
 *   position - 0~32767 对应 0~360°（360° = 满量程 32767）
 *   tag      - 日志里这句话的“标题”，比如 "move to 90deg" / "single move 2/2 -> 360deg"
 */
static uint8_t MotorCtrl_MoveToPosition(uint8_t motor_id, uint16_t position, const char *tag)
{
    MotorValueAck ack;
    char          line[MOTOR_LINE_SIZE];
    char          hex[MOTOR_HEX_SIZE];
    char          fault_text[48];

    /* 升级中：绝对不能让电机转起来（正在擦写 Flash，而且没人看护） */
    if (s_ota_mode != 0U)
    {
        UartLog_Print("motor: OTA in progress, move ignored\r\n");
        return 0U;
    }

    uint8_t ok = Motor_SetValue(motor_id, (int16_t)position,
                                MOTOR_MOVE_ACCEL_TIME, MOTOR_MOVE_BRAKE, &ack);

    MotorCtrl_HexToText(ack.raw, MOTOR_FRAME_SIZE, hex, sizeof(hex));
    Motor_FaultText(ack.fault, fault_text, sizeof(fault_text));

    (void)snprintf(line, sizeof(line),
                   "id=%u %s (position=%u, 0x64): reply=%s, speed=%d, current=%d, "
                   "temp=%uC, fault=0x%02X (%s), rx=%s\r\n",
                   (unsigned int)motor_id, tag, (unsigned int)position,
                   (ok != 0U) ? "OK" : "NONE", (int)ack.speed, (int)ack.current,
                   (unsigned int)ack.temperature, (unsigned int)ack.fault, fault_text, hex);
    UartLog_Print(line);

    return ok;
}

/* ==== 速度环：两轮差速小车用这个环 ==========================================
 *
 * 为什么小车用**速度环（0x02）**而不是位置环 / 电流环：
 *   - 位置环（0x03）：0x64 的给定值是"某一圈里的一个角度"（0~32767 ↔ 0~360°），
 *     而且**按最短路径**走过去、接近目标时慢慢爬（实测 90° 要 1.7~2s 才停稳）。
 *     轮子要的是"一直以某个转速转下去"，位置环只会让车走一小段就停 —— 不适合当驱动。
 *   - 电流环（使能后的默认模式）：给定的是力矩，转速随负载漂（左右轮负载不一样就跑不直），
 *     要自己闭环还得另配一路编码器反馈。
 *   - 速度环：给定值就是转速（±3800 ↔ ±380rpm，1 单位 = 0.1rpm），电机内部自己闭环。
 *     左右轮给定相同、转速就相同，压到地毯/上坡也会自己补力矩 ⇒ 能走直、能定速巡航，
 *     而且给定 0 就是"停"（不用像位置环那样担心 0 被理解成"走到 0°"）。
 *   所以本文件里新增的东西都围绕速度环：切环 + 复核、给转速、加速时间。
 */

/* 总线 ID → s_speed_accel 的下标（= motor_ids[] 的下标，也就是命令里的"几号机"-1）；
   不是本总线上的电机返回 0xFF */
static uint8_t MotorCtrl_IndexOfMotorId(uint8_t motor_id)
{
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        if (motor_ids[i] == motor_id)
        {
            return i;
        }
    }

    return 0xFFU;
}

uint8_t MotorCtrl_GetAccel(uint8_t motor_id)
{
    uint8_t idx = MotorCtrl_IndexOfMotorId(motor_id);

    return (idx == 0xFFU) ? (uint8_t)MOTOR_SPEED_ACCEL_DEFAULT : s_speed_accel[idx];
}

/* 清"已在速度环"的缓存（按总线 ID）。谁离开速度环谁就得清，见 s_in_speed_loop 的注释 */
static void MotorCtrl_ForgetSpeedLoop(uint8_t motor_id)
{
    uint8_t slot = MotorCtrl_IndexOfMotorId(motor_id);

    if (slot != 0xFFU)
    {
        s_in_speed_loop[slot] = 0U;
    }
}

static void MotorCtrl_RememberSpeedLoop(uint8_t motor_id)
{
    uint8_t slot = MotorCtrl_IndexOfMotorId(motor_id);

    if (slot != 0xFFU)
    {
        s_in_speed_loop[slot] = 1U;
    }
}

void MotorCtrl_ForgetSpeedLoopAll(void)
{
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        s_in_speed_loop[i] = 0U;
    }
}

/*
 * 确认第 slot 台在速度环：
 *   缓存命中 → 直接返回（不碰总线）；
 *   缓存没有（第一次 / 失能后 / 断电后 / 切过位置环）→ 走一次
 *   "使能 + 0xA0/0x02 + 0x75 复核"，成功才置位。
 * 返回 1 = 可以发 0x64。
 */
static uint8_t MotorCtrl_EnsureSpeedLoop(uint8_t slot)
{
    if (slot >= MOTOR_COUNT)
    {
        return 0U;
    }

    if (s_in_speed_loop[slot] != 0U)
    {
        return 1U;
    }

    if (MotorCtrl_PrepareSpeedLoop(motor_ids[slot]) != MOTOR_MODE_SPEED)
    {
        UartLog_Print("motor: not in speed loop, 0x64 not sent\r\n");
        return 0U;
    }

    s_in_speed_loop[slot] = 1U;

    return 1U;
}

uint8_t MotorCtrl_SetAccel(uint8_t motor_id, uint8_t ms_per_rpm)
{
    uint8_t idx = MotorCtrl_IndexOfMotorId(motor_id);

    if (idx == 0xFFU)
    {
        return 0U;
    }

    s_speed_accel[idx] = ms_per_rpm;

    return 1U;
}

/*
 * 使能 + 切速度环（0xA0/0x02）+ **复核模式值**。返回复核到的模式值：
 *   MOTOR_MODE_SPEED(0x02) = 成功；其它值 / MOTOR_MODE_UNKNOWN = 没成功。
 *
 * ⚠ 和位置环同一个坑：只看"有没有合法的 0xA1 回帧"不够 —— 电机只要收到了命令就会回
 *   0xA1，回帧里 DATA[2] 才是**切换之后的实际模式**。所以必须用 0x75/0x76 再查一次，
 *   确认真的报 0x02，否则后面按"转速"发 0x64 时它可能还在电流环里（会被当成电流给定）。
 */
uint8_t MotorCtrl_PrepareSpeedLoop(uint8_t motor_id)
{
    MotorMode ack;
    uint8_t   mode_after;
    char      line[MOTOR_LINE_SHORT];

    if (MotorCtrl_Enable(motor_id, &ack) == 0U)
    {
        UartLog_Print("motor: enable FAILED (no valid 0xA1) - check power/wiring\r\n");
        return MOTOR_MODE_UNKNOWN;
    }

    if (MotorCtrl_SetModeRetry(motor_id, MOTOR_MODE_SPEED, &ack,
                              MOTOR_SETMODE_RETRY_MS, MOTOR_SETMODE_RETRY_MAX) == 0U)
    {
        (void)snprintf(line, sizeof(line),
                       "id=%u set speed loop (0xA0/0x02): no valid 0xA1\r\n",
                       (unsigned int)motor_id);
        UartLog_Print(line);
        return MOTOR_MODE_UNKNOWN;
    }

    mode_after = MotorCtrl_QueryModeValue(motor_id);         /* 0x75 -> 0x76 复核 */

    if (mode_after != MOTOR_MODE_SPEED)
    {
        (void)snprintf(line, sizeof(line),
                       "id=%u speed loop NOT active (mode=0x%02X, %s) - refusing\r\n",
                       (unsigned int)motor_id, (unsigned int)mode_after,
                       Motor_ModeName(mode_after));
        UartLog_Print(line);
        return mode_after;
    }

    (void)snprintf(line, sizeof(line), "id=%u speed loop confirmed (0x02)\r\n",
                   (unsigned int)motor_id);
    UartLog_Print(line);

    /* 记住“以后不用再查了” —— 下一次 ctrl speed / DRIVE 直接发 0x64 */
    MotorCtrl_RememberSpeedLoop(motor_id);

    return MOTOR_MODE_SPEED;
}

/*
 * 速度环下给一个转速：发 0x64（给定值 = 转速，单位 0.1rpm，带符号），等 0x65。
 * speed_0p1rpm = 0 就是停（速度环里 0 = 0rpm，不会像位置环那样变成"走到 0°"）。
 *
 * ⚠ 模式检查是**缓存**的（s_in_speed_loop）：确认过一次就不再每条命令查 0x75，
 *   省一趟总线往返。缓存没有时（第一次 / 失能后 / 断电后 / 切过位置环）才走
 *   "使能 + 0xA0/0x02 + 0x75 复核"；0x64 失败也会把缓存清掉，下次自动重新确认 ——
 *   绝不能不知道它在哪个环就发 0x64：同一个数值在电流环里是电流、位置环里是目标位置。
 * 返回 1 = 收到合法 0x65 反馈；0 = 失败（原因在日志里）。
 */
uint8_t MotorCtrl_SetSpeed(uint8_t motor_id, int16_t speed_0p1rpm)
{
    MotorValueAck ack;
    char          line[MOTOR_LINE_SHORT];
    char          hex[MOTOR_HEX_SIZE];
    uint8_t       slot  = MotorCtrl_IndexOfMotorId(motor_id);
    uint8_t       accel = MotorCtrl_GetAccel(motor_id);
    int32_t       v     = (int32_t)speed_0p1rpm;
    unsigned long v_abs = (unsigned long)((v < 0) ? -v : v);

    /* 升级中：绝对不能让电机转起来（正在擦写 Flash，而且没人看护） */
    if (s_ota_mode != 0U)
    {
        UartLog_Print("motor: OTA in progress, speed ignored\r\n");
        return 0U;
    }

    if (slot == 0xFFU)
    {
        (void)snprintf(line, sizeof(line), "id=%u is not on this bus\r\n",
                       (unsigned int)motor_id);
        UartLog_Print(line);
        return 0U;
    }

    if (MotorCtrl_EnsureSpeedLoop(slot) == 0U)
    {
        return 0U;      /* 日志那边已经打过了 */
    }

    uint8_t ok = Motor_SetValue(motor_id, speed_0p1rpm, accel, MOTOR_MOVE_BRAKE, &ack);

    if (ok == 0U)
    {
        s_in_speed_loop[slot] = 0U;     /* 兜底：下次重新确认模式再发 */
    }

    MotorCtrl_HexToText(ack.raw, MOTOR_FRAME_SIZE, hex, sizeof(hex));

    (void)snprintf(line, sizeof(line),
                   "id=%u speed %s%lu.%lurpm (raw=%d, accel=%ums): reply=%s\r\n",
                   (unsigned int)motor_id, (v < 0) ? "-" : "+",
                   v_abs / 10UL, v_abs % 10UL, (int)speed_0p1rpm, (unsigned int)accel,
                   (ok != 0U) ? "OK" : "NONE");
    UartLog_Print(line);

    /* 第二行单独打反馈：速度环下 0x65 的"速度"字段才真的是转速（位置环下那个字段不是速度，
       见 README 的待确认项），current 能看出轮子被压住/顶住了没有 */
    (void)snprintf(line, sizeof(line),
                   "id=%u speed ack: speed=%d, current=%d, temp=%uC, fault=0x%02X, rx=%s\r\n",
                   (unsigned int)motor_id, (int)ack.speed, (int)ack.current,
                   (unsigned int)ack.temperature, (unsigned int)ack.fault, hex);
    UartLog_Print(line);

    return ok;
}

/*
 * 多台**一起**给转速：speeds[i] 对应 motor_ids[i]（= 命令里的 1 / 2 号机）。
 *
 * 为什么要有这个入口（小车起步用）：`ctrl speed` 连发两次 = 两次无线往返 +
 * 每台各一次模式查询，而且两条 0x64 中间夹着上位机的循环和打印 —— 实测两台起步
 * 差 30~80ms，低速时就是"先动的那侧把车拽歪一下"。这里把两条 0x64 **背靠背**
 * 发出去（中间不查模式、不打长日志），差就只剩总线串行的 ~10ms 了。
 *
 * 返回 1 = 全部收到合法 0x65。
 */
uint8_t MotorCtrl_SetSpeeds(const int16_t *speeds, uint8_t count)
{
    MotorValueAck ack;
    char          line[MOTOR_LINE_SHORT];
    uint8_t       ok = 1U;

    if ((speeds == NULL) || (count == 0U) || (count > MOTOR_COUNT))
    {
        return 0U;
    }

    if (s_ota_mode != 0U)
    {
        UartLog_Print("motor: OTA in progress, speed ignored\r\n");
        return 0U;
    }

    /* 先把需要重新确认的**都**确认完，再统一发 0x64：确认这一段的延时两台一样，
       而两条 0x64 中间不夹任何别的事 —— 起步差就只剩总线本身串行的那一点 */
    for (uint8_t i = 0U; i < count; i++)
    {
        if (MotorCtrl_EnsureSpeedLoop(i) == 0U)
        {
            return 0U;      /* 日志已经打过了 */
        }
    }

    for (uint8_t i = 0U; i < count; i++)
    {
        if (Motor_SetValue(motor_ids[i], speeds[i], s_speed_accel[i],
                           MOTOR_MOVE_BRAKE, &ack) == 0U)
        {
            s_in_speed_loop[i] = 0U;    /* 兜底：下次重新确认模式 */
            ok = 0U;
            break;
        }
    }

    if (count >= 2U)
    {
        (void)snprintf(line, sizeof(line),
                       "speed pair: id%u %d, id%u %d (0.1rpm) -> %s\r\n",
                       (unsigned int)motor_ids[0], (int)speeds[0],
                       (unsigned int)motor_ids[1], (int)speeds[1],
                       (ok != 0U) ? "OK" : "NONE");
    }
    else
    {
        (void)snprintf(line, sizeof(line), "speed: id%u %d (0.1rpm) -> %s\r\n",
                       (unsigned int)motor_ids[0], (int)speeds[0],
                       (ok != 0U) ? "OK" : "NONE");
    }

    UartLog_Print(line);

    return ok;
}

/*
 * 单台失能（0xA0/0x09）：把收到的 10 字节原样打出来。
 * 失能后电机只是「不使劲」，通信还在，所以轮询任务照样能查状态/故障码。
 */
static void MotorCtrl_DisableOne(uint8_t motor_id)
{
    MotorMode ack;
    char      line[MOTOR_LINE_SIZE];
    char      hex[MOTOR_HEX_SIZE];

    uint8_t ok = MotorCtrl_Disable(motor_id, &ack);

    /* 失能之后它不再保持任何闭环状态，下次给转速得当新的来：清掉缓存 */
    MotorCtrl_ForgetSpeedLoop(motor_id);

    MotorCtrl_HexToText(ack.raw, MOTOR_FRAME_SIZE, hex, sizeof(hex));

    (void)snprintf(line, sizeof(line),
                   "id=%u disable (0xA0/0x%02X): reply=%s, mode=0x%02X (%s), rx=%s\r\n",
                   (unsigned int)motor_id, (unsigned int)MOTOR_MODE_DISABLE,
                   (ok != 0U) ? "OK" : "NONE", (unsigned int)ack.mode,
                   Motor_ModeName(ack.mode), hex);
    UartLog_Print(line);
}

/*
 * 失能总线上每一台电机（0xA0/0x09）。
 *
 * **不动电机电源（PC14）**：失能只是"不使劲"，通信还在，状态/编码器/故障码照样能读；
 * 断电是另一条命令（OTA_CTRL_PWR / arg=0，上位机 `motor.py pwoff`）。
 * 下次要使能时如果电源还是关的，ENABLE 自己会先上电（MotorPwr_OnAndSettle）。
 */
static void MotorCtrl_DisableAll(void)
{
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        MotorCtrl_DisableOne(motor_ids[i]);
    }
}

/* 查一轮 0x74：里程 / 位置 / 故障码 */
static void MotorCtrl_PollOnce(void)
{
    char line[MOTOR_LINE_SIZE];
    char hex[MOTOR_HEX_SIZE];

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        MotorStatus status;
        char        fault_text[48];

        uint8_t ok = Motor_QueryStatus(motor_ids[i], &status);
        uint16_t deci_deg = Motor_PositionToDeciDeg(status.position);

        Motor_FaultText(status.fault, fault_text, sizeof(fault_text));
        MotorCtrl_HexToText(status.raw, MOTOR_FRAME_SIZE, hex, sizeof(hex));

        (void)snprintf(line, sizeof(line),
                       "id=%u status (0x74): reply=%s, mileage=%ld, position=%u (%u.%udeg), "
                       "fault=0x%02X (%s), rx=%s\r\n",
                       (unsigned int)motor_ids[i], (ok != 0U) ? "OK" : "NONE",
                       (long)status.mileage, (unsigned int)status.position,
                       (unsigned int)(deci_deg / 10U), (unsigned int)(deci_deg % 10U),
                       (unsigned int)status.fault, fault_text, hex);
        UartLog_Print(line);
    }
}

/*
 * 轮询任务：等定时器给的信号量，来一个查一轮。
 * 信号量是二元的（上限 1），所以轮询任务忙的时候定时器多按几次也只会合并成一次，
 * 不会积压出一串迟到几百 ms 的查询。
 */
static void MotorCtrl_PollTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        if (osSemaphoreAcquire(s_poll_sem, osWaitForever) == osOK)
        {
            MotorCtrl_PollOnce();
        }
    }
}

/* Exported functions --------------------------------------------------------*/

void MotorCtrl_Init(void)
{
    /* 速度环加速时间的上电默认值（每台一份，见 s_speed_accel） */
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        s_speed_accel[i] = MOTOR_SPEED_ACCEL_DEFAULT;
    }

    if (s_poll_sem == NULL)
    {
        s_poll_sem = osSemaphoreNew(1U, 0U, NULL);
    }

    if (s_poll_task == NULL)
    {
        s_poll_task = osThreadNew(MotorCtrl_PollTask, NULL, &s_poll_task_attr);
    }
}

void MotorCtrl_OnPollTimer(void)
{
    if (s_poll_sem != NULL)
    {
        /* 只做这一件事。释放信号量不会阻塞，在 timer service task 里是安全的 */
        (void)osSemaphoreRelease(s_poll_sem);
    }
}

/* ---- 无线控制入口（OtaService 用：OTA 平时就是靠这个口传控制信息） -------- */

/*
 * 进入/退出"OTA 模式"：
 *   进入 = 电机全部失能 + 停 200 ms 轮询 + 禁止后续走动命令（不动电源）
 *   退出 = 恢复轮询（电机仍然保持失能，要动就再发 ENABLE）
 */
void MotorCtrl_SetOtaMode(uint8_t on)
{
    if ((on != 0U) && (s_ota_mode == 0U))
    {
        s_ota_mode = 1U;

        MotorCtrl_PollPause(1U);                      /* 先停轮询，别和 OTA 抢带宽 */

        /* 小车动作也停掉：升级中途在跑闭环是不可接受的 */
        (void)MotorDrive_Stop();

        MotorCtrl_DisableAll();                       /* 最后失能：升级期间电机必须不使劲 */
        MotorCtrl_ForgetSpeedLoopAll();               /* 失能了，速度环缓存也一起作废 */
    }
    else if ((on == 0U) && (s_ota_mode != 0U))
    {
        s_ota_mode = 0U;

        MotorCtrl_PollPause(0U);
    }
}

uint8_t MotorCtrl_OtaMode(void)
{
    return s_ota_mode;
}

/*
 * 无线控制命令：上位机发 OTA_T_CTRL，payload[0] = cmd、payload[1..4] = arg。
 * 返回 OTA_OK 或 OTA_E_xxx（见 ota_layout.h），*out 是给上位机看的附加信息。
 */
uint8_t MotorCtrl_RemoteCmd(uint8_t motor_index, uint8_t cmd, uint32_t arg,
                            uint32_t *out, uint32_t *out2, uint32_t *out3)
{
    MotorMode ack;
    uint8_t   ok = 1U;

    /* motor_index：0 = 没指定（安全类命令作用于全部，运动类默认 1 号机）；
                    1..MOTOR_COUNT = 指定那一台（= motor_ids[] 里的下标 +1）
       ⚠ OTA_CTRL_DRIVE 是例外：它把第 6 个字节当**看门狗超时**用（不是电机序号），
         所以下面的序号检查与下面的"作用于谁"都跳过它单独处理 */
    uint8_t   motor_id = MotorCtrl_MotorIdOfIndex((motor_index == 0U) ? 1U : motor_index);

    if ((cmd != OTA_CTRL_DRIVE) && (motor_index > MOTOR_COUNT))
    {
        return OTA_E_PARAM;    /* 指定了一个没有的电机序号 */
    }

    if (out != NULL)
    {
        *out = 0U;
    }

    if (out2 != NULL)
    {
        *out2 = 0U;
    }

    if (out3 != NULL)
    {
        *out3 = 0U;
    }

    /* 升级期间只允许"失能 / 急停 / 静音 / 停轮询 / 断电"，别的都拒绝 ——
       正在擦写 Flash 的时候让电机转起来是找死；
       电源也只准"关"，不许在升级期间把电机重新上电。 */
    if ((s_ota_mode != 0U) && (cmd != OTA_CTRL_DISABLE) && (cmd != OTA_CTRL_STOP) &&
        (cmd != OTA_CTRL_LOG_MUTE) && (cmd != OTA_CTRL_POLL_PAUSE) &&
        !((cmd == OTA_CTRL_PWR) && (arg == 0U)))
    {
        return OTA_E_NOTALLOWED;
    }

    switch (cmd)
    {
        case OTA_CTRL_DISABLE:
            /* 只失能，不碰电机电源（断电是 OTA_CTRL_PWR 的事）；
               不指定电机 = 总线上全部失能，指定了 = 只失能那一台 */
            if (motor_index == 0U)
            {
                MotorCtrl_DisableAll();
            }
            else
            {
                MotorCtrl_DisableOne(motor_id);
            }
            break;

        case OTA_CTRL_ENABLE:
            /* 同按键流程：先上电等稳定，再发使能帧（否则前面几次重试都是白跑） */
            MotorPwr_OnAndSettle();

            if (MotorCtrl_Enable(motor_id, &ack) == 0U)
            {
                ok = 0U;
            }
            break;

        case OTA_CTRL_POS_LOOP:
            /* 只看回帧不够（不认 0x03 的固件也会回合法的 0xA1），必须确认模式值真是 0x03 */
            if ((MotorCtrl_EnterPositionLoop(motor_id, &ack) == 0U) ||
                (ack.mode != MOTOR_MODE_POSITION))
            {
                ok = 0U;
            }
            break;

        case OTA_CTRL_MOVE_POS:
            if (arg > 32767U)
            {
                return OTA_E_PARAM;
            }
            if (MotorCtrl_MoveToPosition(motor_id, (uint16_t)arg, "remote move (0x64)") == 0U)
            {
                ok = 0U;
            }
            break;

        case OTA_CTRL_MOVE_DEG:
            if (arg > 359U)
            {
                return OTA_E_PARAM;
            }
            if (MotorCtrl_MoveToPosition(motor_id, Motor_AngleToPosition((uint16_t)arg),
                                         "remote move (deg)") == 0U)
            {
                ok = 0U;
            }
            break;

        case OTA_CTRL_STOP:
            /* 不指定 = 全部急停（安全默认）；指定了只停那一台 */
            if (motor_index == 0U)
            {
                for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
                {
                    MotorCtrl_Stop(motor_ids[i]);      /* 0x64 给定值 0 = 急停 */
                }
            }
            else
            {
                MotorCtrl_Stop(motor_id);
            }
            break;

        case OTA_CTRL_LOG_MUTE:
            UartLog_SetEnabled((arg != 0U) ? 0U : 1U);
            break;

        case OTA_CTRL_POLL_PAUSE:
            /* 用可嵌套的暂停接口（别直接 osTimerStop）：OTA 也在用同一个定时器 */
            MotorCtrl_PollPause((arg != 0U) ? 1U : 0U);
            break;

        case OTA_CTRL_SPEED_LOOP:
            /* 切速度环（小车用这个环）：先上电等稳定（没上电时前面几次重试都是白跑），
               再使能 + 0xA0/0x02 + 复核模式值。
               回复 data = 复核到的模式值（0x02 = 成功）、data2 = 这一台的总线 ID */
        {
            uint8_t mode_now;

            MotorPwr_OnAndSettle();

            mode_now = MotorCtrl_PrepareSpeedLoop(motor_id);

            if (mode_now != MOTOR_MODE_SPEED)
            {
                ok = 0U;
            }

            if (out != NULL)
            {
                *out = mode_now;
            }

            if (out2 != NULL)
            {
                *out2 = motor_id;
            }
            break;
        }

        case OTA_CTRL_SPEED:
            /* 转速给定：arg 是**带符号**的 0.1rpm（-3800~3800 ↔ -380~380rpm），0 = 停。
               上位机发负数时按 32 位补码过来，先还原成 int16 */
        {
            int16_t speed = (int16_t)arg;

            if (((int32_t)speed > MOTOR_SPEED_MAX_RAW) ||
                ((int32_t)speed < -MOTOR_SPEED_MAX_RAW))
            {
                return OTA_E_PARAM;
            }

            MotorPwr_OnAndSettle();     /* 没上电的话下面第一次 0x75 就是白问 */

            if (MotorCtrl_SetSpeed(motor_id, speed) == 0U)
            {
                ok = 0U;
            }

            if (out != NULL)
            {
                *out = (uint32_t)(int32_t)speed;    /* 回帧里原样回生效值（带符号） */
            }

            if (out2 != NULL)
            {
                *out2 = motor_id;
            }
            break;
        }

        case OTA_CTRL_ACCEL:
            /* 加速时间（每 1rpm 多少 ms，0..255）：存在 RAM 里，speed 命令立刻生效 */
            if (arg > 255U)
            {
                return OTA_E_PARAM;
            }

            if (MotorCtrl_SetAccel(motor_id, (uint8_t)arg) == 0U)
            {
                ok = 0U;
            }

            if (out != NULL)
            {
                *out = MotorCtrl_GetAccel(motor_id);
            }

            if (out2 != NULL)
            {
                *out2 = motor_id;
            }
            break;

        case OTA_CTRL_PWR:
            /* arg：0 = 断电，1 = 上电（等稳定），2 = 断电重启（电机状态卡死时用） */
            if (arg == 0U)
            {
                MotorPwr_Enable(0U);
            }
            else if (arg == 1U)
            {
                MotorPwr_OnAndSettle();
            }
            else if (arg == 2U)
            {
                MotorPwr_PowerCycle();
            }
            else
            {
                return OTA_E_PARAM;
            }

            /* 断电/重启之后电机重新上电，默认回到电流环：速度环缓存作废
               （不断电的那种只是等稳定，不用清） */
            if (arg != 1U)
            {
                MotorCtrl_ForgetSpeedLoopAll();
            }

            if (out != NULL)
            {
                *out = MotorPwr_IsOn();    /* 告诉上位机现在到底是开是关 */
            }
            break;

        case OTA_CTRL_DRIVE:
            /* 小车动作：两台一起给基准转速（背靠背两条 0x64），看门狗交给 motor_drive 那个循环。
               ⚠ 这条命令的 payload 第 6 字节在那一边读出来是 motor_index，但它的含义是
                 **看门狗超时**（单位 50ms，0 = 不启用），不是电机序号 ——
                 所以上面那个"序号越界"检查对它是绕开的。 */
        {
            int16_t  v[MOTOR_COUNT];
            uint32_t wd_ms = (uint32_t)motor_index * MOTOR_DRIVE_WD_UNIT_MS;

            v[0] = (int16_t)(arg & 0xFFFFU);
            v[1] = (int16_t)(arg >> 16);

            if (((v[0] > MOTOR_SPEED_MAX_RAW) || (v[0] < -MOTOR_SPEED_MAX_RAW)) ||
                ((v[1] > MOTOR_SPEED_MAX_RAW) || (v[1] < -MOTOR_SPEED_MAX_RAW)))
            {
                return OTA_E_PARAM;
            }

            MotorPwr_OnAndSettle();     /* 没上电什么也发不出去（已经是开的时候不会白等 500ms） */

            if (MotorDrive_Set(v, MOTOR_COUNT, wd_ms) == 0U)
            {
                ok = 0U;
            }

            if (out != NULL)
            {
                *out = arg;             /* 原样回两个基准，上位机好对账 */
            }

            if (out2 != NULL)
            {
                *out2 = wd_ms;
            }
            break;
        }

        case OTA_CTRL_TRIM:
            /* 里程差闭环：arg = 0 关 / 1..MOTOR_COUNT = 开，并且这个数就是"左轮"的电机序号 */
        {
            uint8_t on   = 0U;
            uint8_t left = 0U;
            int32_t trim = 0;

            if (arg > MOTOR_COUNT)
            {
                return OTA_E_PARAM;
            }

            if (MotorDrive_SetTrim((uint8_t)arg) == 0U)
            {
                ok = 0U;
            }

            MotorDrive_GetTrim(&on, &left, &trim);

            if (out != NULL)
            {
                *out = on;                              /* 0/1：闭环开没开 */
            }

            if (out2 != NULL)
            {
                *out2 = left;                           /* 左轮是几号机（0 = 没配） */
            }

            if (out3 != NULL)
            {
                *out3 = (uint32_t)trim;                 /* 当前 trim 输出（0.1rpm，带符号） */
            }
            break;
        }

        default:
            return OTA_E_PARAM;
    }

    return (ok != 0U) ? (uint8_t)OTA_OK : (uint8_t)OTA_E_STATE;
}

/*
 * 只读状态快照：0x74（里程/位置/故障码）+ 0x75（当前模式）。
 * 比"一问一答 + 文本日志"更适合上位机画曲线。
 */
uint8_t MotorCtrl_RemoteStatus(uint8_t motor_index, int32_t *mileage, uint16_t *position,
                               uint8_t *fault, uint8_t *mode)
{
    MotorStatus status;
    MotorMode   mode_ack;
    uint8_t     motor_id = MotorCtrl_MotorIdOfIndex((motor_index == 0U) ? 1U : motor_index);

    if ((mileage == NULL) || (position == NULL) || (fault == NULL) || (mode == NULL))
    {
        return OTA_E_PARAM;
    }

    if ((motor_index > MOTOR_COUNT) || (motor_id == 0U))
    {
        return OTA_E_PARAM;    /* 指定了一个没有的电机序号 */
    }

    if (Motor_QueryStatus(motor_id, &status) == 0U)
    {
        return OTA_E_STATE;    /* 电机没答上（没上电/没接线） */
    }

    *mileage  = status.mileage;
    *position = status.position;
    *fault    = status.fault;

    if (Motor_QueryMode(motor_id, &mode_ack) != 0U)
    {
        *mode = mode_ack.mode;
    }
    else
    {
        *mode = MOTOR_MODE_UNKNOWN;
    }

    return OTA_OK;
}

/* ---- 内部工具（无线命令入口用） ---- */

/*
 * 查一次模式值（0x75 -> 0x76），**不打日志**：切环流程里“当前到底是什么模式”
 * 这种判断不能每次都往日志里灌。查不到（超时/不答）返回 MOTOR_MODE_UNKNOWN。
 */
uint8_t MotorCtrl_QueryModeValue(uint8_t motor_id)
{
    MotorMode mode;

    return (Motor_QueryMode(motor_id, &mode) != 0U) ? mode.mode : (uint8_t)MOTOR_MODE_UNKNOWN;
}

/*
 * 暂停 / 恢复 200 ms 状态轮询。**用计数而不是布尔**：
 * OTA 会话和别的地方都要把它停掉，都恢复了才该重新开始 ——
 * 布尔值的话，先恢复的那一个会把另一个还需要的“暂停”一并解开。
 */
void MotorCtrl_PollPause(uint8_t on)
{
    if (on != 0U)
    {
        if (s_poll_pause_cnt < 0xFFU)
        {
            s_poll_pause_cnt++;
        }

        (void)osTimerStop(motorPosHandle);
    }
    else if (s_poll_pause_cnt > 0U)
    {
        s_poll_pause_cnt--;

        if (s_poll_pause_cnt == 0U)
        {
            (void)osTimerStart(motorPosHandle, pdMS_TO_TICKS(MOTOR_CTRL_POLL_MS));
        }
    }
}

