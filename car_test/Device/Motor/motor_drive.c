/**
  ******************************************************************************
  * @file    motor_drive.c
  * @brief   小车底盘驱动：里程差闭环（走直线）+ 看门狗
  *
  * 一拍干四件事：
  *   1) 读两台 0x74（位置）—— 一带闭环用，二当"任务还活着"的证明；
  *   2) 位置增量之差 → 积分 → trim（只在"直行类"动作下积分，见 .h 的说明）；
  *   3) 目标 = 基准 ± trim，**只在变化 ≥1 个单位时才发**（0x64 是"给了就一直保持"，
  *      没必要每拍重发：省下的总线时间正好用来多读几拍位置）；
  *   4) 看门狗：超时就把两台停掉（松手/断线/上位机挂了都靠它兜底）。
  *
  * ⚠ 位置只用"最短路径增量"累加，不要用 0x74 的里程字段（它过零点时会掉一整圈，
  *   详见 motor_follow.c 里 MotorFollow_ShortestDiff 的注释）。
  * ⚠ 不在这里点 LED：WS2812 走 SPI6，只允许一个任务驱动（现有约定是 MotorCtrl_Task 独占）。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_drive.h"

#include "motor.h"
#include "motor_ctrl.h"     /* MotorCtrl_SetSpeeds / MotorCtrl_MotorIdOfIndex / PollPause */
#include "uart_log.h"

#include "FreeRTOS.h"
#include "task.h"           /* uxTaskGetStackHighWaterMark() */
#include "cmsis_os2.h"

#include <stdio.h>

/* Private define ------------------------------------------------------------*/

/* 一圈的计数：位置值 0~32767 是一圈（32767 == 0，同一个点）⇒ 32768 */
#define DRIVE_COUNTS_PER_TURN     32768

/* 日志行缓冲：这里的行都短（最长 ~110 字节），给 160 足够 */
#define DRIVE_LINE_SIZE           160U

/* 任务栈（字节，CMSIS-RTOS2 的 stack_size 单位是字节）：
   里面有 snprintf（日志行）+ HAL 调用，和跟随/轮询任务一样留 2 KB */
#define DRIVE_TASK_STACK          (512U * 4U)

/* 状态行周期（1 Hz，和跟随一样，不用另开终端也够看） */
#define DRIVE_REPORT_MS           1000U

/* Private variables ---------------------------------------------------------*/

static osThreadId_t    s_task = NULL;
static osSemaphoreId_t s_sem  = NULL;

static const osThreadAttr_t s_task_attr =
{
    .name       = "motorDrive",
    .stack_size = DRIVE_TASK_STACK,
    .priority   = (osPriority_t)osPriorityNormal,
};

/* 1 = 循环该跑；0 = 该退出 */
static volatile uint8_t s_run  = 0U;

/* 1 = 正在"一拍"里（可能正在用总线）。Stop() 靠它保证自己发的命令
   不会被循环里那一拍覆盖掉 */
static volatile uint8_t s_busy = 0U;

/* 1 = 200 ms 轮询是我们停的（只能由我们恢复）：
   PollPause 是个共享计数器，别的模块（OTA/跟随）也在用，
   自己不记着的话，一次"替别人恢复"就会把轮询提前开回来抢总线 */
static uint8_t s_poll_paused = 0U;

/* ---- 给定 ---- */
static int16_t  s_base[MOTOR_COUNT];        /* 上位机给的基准转速（0.1rpm，正 = 往前推） */
static int16_t  s_sent[MOTOR_COUNT];        /* 上一拍真正发出去的（含 trim） */
static uint8_t  s_sent_valid = 0U;          /* 0 = 还没发过（下一拍必发） */
static uint32_t s_watchdog_ms = 0U;         /* 0 = 不启用 */
static uint32_t s_last_cmd_ms = 0U;         /* 收到最近一条 DRIVE 的时刻（ms） */

/* ---- 里程差闭环 ---- */
static uint8_t  s_left_slot  = 0xFFU;       /* 左轮是第几台（0/1）；0xFF = 还没配过 */
static uint8_t  s_trim_on    = 0U;          /* 闭环总开关 */
static int32_t  s_err        = 0;           /* 里程差积分（计数，左−右） */
static int32_t  s_trim       = 0;           /* 当前 trim 输出（0.1rpm，加在"左−右"上） */
static uint16_t s_prev_pos[MOTOR_COUNT];    /* 上一拍的位置值 */
static uint8_t  s_have_prev  = 0U;          /* 0 = 还没有上一拍（第一拍只取基准） */
static uint8_t  s_miss[MOTOR_COUNT];        /* 连续读不到的拍数 */

/* ---- 统计（1 Hz 打一行） ---- */
static uint32_t s_window_ms  = 0U;
static uint32_t s_ticks      = 0U;
static uint32_t s_start_ms   = 0U;

/* Private function prototypes -----------------------------------------------*/
static void    MotorDrive_Task(void *argument);
static void    MotorDrive_Step(void);
static void    MotorDrive_Report(void);
static void    MotorDrive_StopMotors(const char *why);
static void    MotorDrive_Finish(const char *why);
static uint8_t MotorDrive_TrimApplies(void);

/* Private functions ---------------------------------------------------------*/

/* tick → ms（本工程 1 tick = 1 ms，仍然按 osKernelGetTickFreq() 算，免得以后改 tick 率时静默算错） */
static uint32_t MotorDrive_TicksToMs(uint32_t ticks)
{
    uint32_t freq = osKernelGetTickFreq();

    if (freq == 0U)
    {
        freq = 1000U;
    }

    return (uint32_t)(((uint64_t)ticks * 1000ULL) / (uint64_t)freq);
}

static uint32_t MotorDrive_NowMs(void)
{
    return MotorDrive_TicksToMs(osKernelGetTickCount());
}

/*
 * 两次位置值之间"实际走了多少计数"：位置一圈是 0~32767 且 32767 与 0 是同一点，
 * 所以按**最短路径**算，范围 -16384..+16384。
 * ⚠ 前提是一拍转不超过 180°：本工程一拍 25 ms、电机最快 380 rpm（2280°/s）⇒ 最多 57°，安全。
 */
static int32_t MotorDrive_ShortestDiff(uint16_t from, uint16_t to)
{
    int32_t d = (int32_t)to - (int32_t)from;

    if (d > (DRIVE_COUNTS_PER_TURN / 2))
    {
        d -= DRIVE_COUNTS_PER_TURN;
    }
    else if (d < -(DRIVE_COUNTS_PER_TURN / 2))
    {
        d += DRIVE_COUNTS_PER_TURN;
    }

    return d;
}

/*
 * 现在这个给定算不算"直行类"（值得让里程差闭环去修）：
 *   两侧同号（都在往前或都在往后）、都非零、且速度差不超过快的那侧的 1/4。
 *   转圈（一正一负）和明显走弧（比如 30/10）都返回 0 —— 那些动作下航向本来就在变，
 *   拿"里程差"当误差去修会跟驾驶指令打架。停止（两侧都 0）也不算。
 */
static uint8_t MotorDrive_TrimApplies(void)
{
    int32_t a = s_base[0];
    int32_t b = s_base[1];
    int32_t d;
    int32_t m;

    if ((a == 0) || (b == 0))
    {
        return 0U;
    }

    if ((a > 0) != (b > 0))
    {
        return 0U;
    }

    if (a < 0) { a = -a; }
    if (b < 0) { b = -b; }

    d = (a > b) ? (a - b) : (b - a);
    m = (a > b) ? a : b;

    return (d * (int32_t)MOTOR_DRIVE_STRAIGHT_DIV <= m) ? 1U : 0U;
}

/* 两台一起停（0x64 给定值 0）。速度环里 0 就是 0rpm，不会像位置环那样变成"走到 0°" */
static void MotorDrive_StopMotors(const char *why)
{
    int16_t zero[MOTOR_COUNT] = { 0, 0 };

    if (MotorCtrl_SetSpeeds(zero, MOTOR_COUNT) != 0U)
    {
        s_sent[0] = 0;
        s_sent[1] = 0;
        s_sent_valid = 1U;
    }
    else
    {
        char line[DRIVE_LINE_SIZE];

        (void)snprintf(line, sizeof(line), "[drive] %s: stop frame FAILED (motor not answering?)\r\n",
                       why);
        UartLog_Print(line);
    }
}

/* 结束本次驱动：停电机、恢复轮询、清状态（Stop() 和看门狗走同一条路） */
static void MotorDrive_Finish(const char *why)
{
    char line[DRIVE_LINE_SIZE];

    MotorDrive_StopMotors(why);

    s_run      = 0U;
    s_trim     = 0;
    s_err      = 0;
    s_sent_valid = 0U;

    if (s_poll_paused != 0U)
    {
        s_poll_paused = 0U;
        MotorCtrl_PollPause(0U);        /* 只恢复"我们自己停的那一次" */
    }

    (void)snprintf(line, sizeof(line), "[drive] %s: t=%lus ticks=%lu\r\n",
                   why, (unsigned long)((MotorDrive_NowMs() - s_start_ms) / 1000U),
                   (unsigned long)s_ticks);
    UartLog_Print(line);
}

/* 1 Hz 状态行：rate 是实测拍频（说明总线还跟得上），e/trim 就是闭环在干什么 */
static void MotorDrive_Report(void)
{
    char     line[DRIVE_LINE_SIZE];
    uint32_t now = MotorDrive_NowMs();
    uint32_t dt  = now - s_window_ms;

    if (dt == 0U)
    {
        return;
    }

    (void)snprintf(line, sizeof(line),
                   "[drive] rate=%luHz base=%d/%d sent=%d/%d trim=%d e=%ld miss=%u/%u\r\n",
                   (unsigned long)(((uint64_t)s_ticks * 1000ULL) / (uint64_t)dt),
                   (int)s_base[0], (int)s_base[1], (int)s_sent[0], (int)s_sent[1],
                   (int)s_trim, (long)s_err, (unsigned int)s_miss[0], (unsigned int)s_miss[1]);
    UartLog_Print(line);

    s_window_ms = now;
    s_ticks     = 0U;
}

/*
 * 一拍。顺序是有讲究的：
 *   看门狗 → 读位置 → 算 trim → 发（只在变化时）→ 统计。
 * 看门狗放最前面：它管的是"别再转了"，比多看一次位置重要。
 */
static void MotorDrive_Step(void)
{
    uint16_t pos[MOTOR_COUNT];
    int32_t  delta[MOTOR_COUNT];
    int16_t  v[MOTOR_COUNT];
    uint8_t  got = 0U;
    uint32_t now = MotorDrive_NowMs();

    /* ---- 0) 看门狗：超时就停，整个驱动结束 ---- */
    if ((s_watchdog_ms != 0U) && ((now - s_last_cmd_ms) > s_watchdog_ms))
    {
        char line[DRIVE_LINE_SIZE];

        (void)snprintf(line, sizeof(line),
                       "[drive] WATCHDOG: no command for %lums -> stop\r\n",
                       (unsigned long)(now - s_last_cmd_ms));
        UartLog_Print(line);

        MotorDrive_Finish("watchdog stop");
        return;
    }

    /* ---- 1) 读两台位置（0x74） ---- */
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        MotorStatus st;

        pos[i]   = s_prev_pos[i];
        delta[i] = 0;

        if (Motor_QueryStatus(MotorCtrl_MotorIdOfIndex((uint8_t)(i + 1U)), &st) != 0U)
        {
            pos[i] = st.position;
            s_miss[i] = 0U;
            got++;
        }
        else if (s_miss[i] < 0xFFU)
        {
            s_miss[i]++;
        }
    }

    if ((s_miss[0] >= MOTOR_DRIVE_MISS_LIMIT) || (s_miss[1] >= MOTOR_DRIVE_MISS_LIMIT))
    {
        UartLog_Print("[drive] motor not answering (0x74) - stop\r\n");
        MotorDrive_Finish("no answer");
        return;
    }

    /* ---- 2) 位置增量 → 里程差积分 → trim ---- */
    if (got == MOTOR_COUNT)
    {
        for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
        {
            delta[i] = (s_have_prev != 0U) ? MotorDrive_ShortestDiff(s_prev_pos[i], pos[i]) : 0;
            s_prev_pos[i] = pos[i];
        }

        s_have_prev = 1U;

        if ((s_trim_on != 0U) && (s_left_slot < MOTOR_COUNT) && (MotorDrive_TrimApplies() != 0U))
        {
            uint8_t l = s_left_slot;
            uint8_t r = (uint8_t)(1U - s_left_slot);

            /* e 正 = 左轮走得多 = 车头往右偏 ⇒ 下一拍要"压左、抬右"（trim 正） */
            s_err += (delta[l] - delta[r]);
            s_err -= s_err / MOTOR_DRIVE_TRIM_LEAK;         /* 泄漏：约 1.6 s 时间常数 */

            if (s_err > MOTOR_DRIVE_ERR_MAX)  { s_err = MOTOR_DRIVE_ERR_MAX; }
            if (s_err < -MOTOR_DRIVE_ERR_MAX) { s_err = -MOTOR_DRIVE_ERR_MAX; }

            s_trim = s_err / MOTOR_DRIVE_TRIM_DIV;

            if (s_trim > MOTOR_DRIVE_TRIM_MAX)  { s_trim = MOTOR_DRIVE_TRIM_MAX; }
            if (s_trim < -MOTOR_DRIVE_TRIM_MAX) { s_trim = -MOTOR_DRIVE_TRIM_MAX; }
        }
        else
        {
            /* 转圈/走弧/停止/没开闭环：修正量归零（积分只做泄漏，不清零 ——
               免得回到直行时又从头积） */
            s_err -= s_err / MOTOR_DRIVE_TRIM_LEAK;
            s_trim = 0;
        }
    }

    /* ---- 3) 目标 = 基准 ± trim，只在变化时发 ---- */
    v[0] = s_base[0];
    v[1] = s_base[1];

    if ((s_trim_on != 0U) && (s_left_slot < MOTOR_COUNT))
    {
        v[s_left_slot]          = (int16_t)(v[s_left_slot] - s_trim);
        v[(uint8_t)(1U - s_left_slot)] = (int16_t)(v[(uint8_t)(1U - s_left_slot)] + s_trim);
    }

    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        if (v[i] > MOTOR_SPEED_MAX_RAW)  { v[i] = MOTOR_SPEED_MAX_RAW; }
        if (v[i] < -MOTOR_SPEED_MAX_RAW) { v[i] = -MOTOR_SPEED_MAX_RAW; }
    }

    if ((s_sent_valid == 0U) || (v[0] != s_sent[0]) || (v[1] != s_sent[1]))
    {
        if (MotorCtrl_SetSpeeds(v, MOTOR_COUNT) != 0U)
        {
            s_sent[0] = v[0];
            s_sent[1] = v[1];
            s_sent_valid = 1U;
        }
        else
        {
            /* 发不出去（没答上/不在速度环）：让下一拍再试，别装作发过了 */
            s_sent_valid = 0U;
        }
    }

    /* ---- 4) 统计 ---- */
    s_ticks++;

    if ((now - s_window_ms) >= DRIVE_REPORT_MS)
    {
        MotorDrive_Report();
    }
}

/*
 * 驱动任务：等信号量 → 按固定节拍跑（osDelayUntil，不是"工作+延时"，
 * 否则实际周期会被每拍的工作时间拖长）→ s_run 清掉就退出去等下一次 Start。
 */
static void MotorDrive_Task(void *argument)
{
    (void)argument;

    for (;;)
    {
        if (osSemaphoreAcquire(s_sem, osWaitForever) != osOK)
        {
            continue;
        }

        uint32_t next = osKernelGetTickCount();

        while (s_run != 0U)
        {
            s_busy = 1U;
            MotorDrive_Step();
            s_busy = 0U;

            if (s_run == 0U)
            {
                break;
            }

            next += pdMS_TO_TICKS(MOTOR_DRIVE_PERIOD_MS);

            if ((int32_t)(osKernelGetTickCount() - next) >= 0)
            {
                /* 一拍没跑完就该进下一拍了：从"现在"重新排，别让周期悄悄变大 */
                next = osKernelGetTickCount();
            }

            (void)osDelayUntil(next);
        }
    }
}

/* Exported functions --------------------------------------------------------*/

uint8_t MotorDrive_Init(void)
{
    if (s_sem == NULL)
    {
        s_sem = osSemaphoreNew(1U, 0U, NULL);
    }

    if (s_task == NULL)
    {
        char line[112];

        s_task = osThreadNew(MotorDrive_Task, NULL, &s_task_attr);

        if (s_task == NULL)
        {
            (void)snprintf(line, sizeof(line),
                           "[drive] task create FAILED (free heap=%lu)\r\n",
                           (unsigned long)xPortGetFreeHeapSize());
            UartLog_Print(line);
            return 0U;
        }
    }

    return ((s_sem != NULL) && (s_task != NULL)) ? 1U : 0U;
}

uint8_t MotorDrive_Set(const int16_t *speeds, uint8_t count, uint32_t watchdog_ms)
{
    char    line[DRIVE_LINE_SIZE];
    uint8_t starting = (s_run == 0U) ? 1U : 0U;

    if ((speeds == NULL) || (count == 0U) || (count > MOTOR_COUNT))
    {
        return 0U;
    }

    if (MotorCtrl_OtaMode() != 0U)
    {
        UartLog_Print("[drive] refused: OTA in progress\r\n");
        return 0U;
    }

    /* 全 0 = 停车（上位机不用为"松手"再单独发一条 stop） */
    if ((speeds[0] == 0) && ((count < 2U) || (speeds[1] == 0)))
    {
        if (starting == 0U)
        {
            MotorDrive_Finish("stop (zero command)");
        }
        else
        {
            MotorDrive_StopMotors("stop (zero command)");   /* 没在跑也照样发一次 0，防呆 */
        }

        return 1U;
    }

    if (MotorDrive_Init() == 0U)
    {
        return 0U;
    }

    for (uint8_t i = 0U; i < count; i++)
    {
        s_base[i] = speeds[i];
    }

    s_watchdog_ms = watchdog_ms;
    s_last_cmd_ms = MotorDrive_NowMs();

    if (starting != 0U)
    {
        s_err        = 0;
        s_trim       = 0;
        s_have_prev  = 0U;
        s_sent_valid = 0U;
        s_miss[0]    = 0U;
        s_miss[1]    = 0U;
        s_ticks      = 0U;
        s_start_ms   = s_last_cmd_ms;
        s_window_ms  = s_start_ms;

        /* 200 ms 状态轮询停掉：它也在这条总线上，跟着一起抢只会让闭环变慢 */
        if (s_poll_paused == 0U)
        {
            s_poll_paused = 1U;
            MotorCtrl_PollPause(1U);
        }

        (void)snprintf(line, sizeof(line),
                       "[drive] start: base %d/%d (0.1rpm), trim=%s (left=%u), wd=%lums\r\n",
                       (int)s_base[0], (int)s_base[1],
                       (s_trim_on != 0U) ? "on" : "off",
                       (unsigned int)((s_left_slot < MOTOR_COUNT) ? (s_left_slot + 1U) : 0U),
                       (unsigned long)watchdog_ms);
        UartLog_Print(line);
        UartLog_Print("[drive]   one line every 1 s; `ctrl trim 0|1|2` to toggle the odometry loop"
                      "\r\n");

        s_run = 1U;
        (void)osSemaphoreRelease(s_sem);
    }

    return 1U;
}

uint8_t MotorDrive_Stop(void)
{
    if (s_run == 0U)
    {
        return 1U;      /* 本来就没跑：当成已经是停的 */
    }

    s_run = 0U;

    /* 等当前那一拍跑完再发停：s_run 已经清零 ⇒ 它下一拍开头就会返回，
       所以我们后面发的命令不会被它覆盖 */
    if (osThreadGetId() != s_task)
    {
        uint32_t guard = 0U;

        while ((s_busy != 0U) && (guard < 100U))
        {
            osDelay(1U);
            guard++;
        }
    }

    MotorDrive_Finish("stop");

    return 1U;
}

uint8_t MotorDrive_IsRunning(void)
{
    return s_run;
}

uint8_t MotorDrive_SetTrim(uint8_t left_index)
{
    char line[DRIVE_LINE_SIZE];

    if (left_index > MOTOR_COUNT)
    {
        return 0U;
    }

    if (left_index == 0U)
    {
        s_trim_on = 0U;
        s_trim    = 0;
        s_err     = 0;
        UartLog_Print("[drive] trim off (odometry loop disabled)\r\n");
        return 1U;
    }

    if ((s_run != 0U) && (left_index != (uint8_t)(s_left_slot + 1U)))
    {
        /* 跑着改"哪台是左"会让车突然反向修一下：先停了再改 */
        UartLog_Print("[drive] trim: refusing to change which wheel is left while running - stop first\r\n");
        return 0U;
    }

    s_left_slot = (uint8_t)(left_index - 1U);
    s_trim_on   = 1U;
    s_err       = 0;
    s_trim      = 0;

    (void)snprintf(line, sizeof(line), "[drive] trim on: left = motor %u (bus id%u)\r\n",
                   (unsigned int)left_index,
                   (unsigned int)MotorCtrl_MotorIdOfIndex(left_index));
    UartLog_Print(line);

    return 1U;
}

void MotorDrive_GetTrim(uint8_t *enabled, uint8_t *left_index, int32_t *trim_out)
{
    if (enabled != NULL)
    {
        *enabled = s_trim_on;
    }

    if (left_index != NULL)
    {
        *left_index = (s_left_slot < MOTOR_COUNT) ? (uint8_t)(s_left_slot + 1U) : 0U;
    }

    if (trim_out != NULL)
    {
        *trim_out = s_trim;
    }
}
