/**
  ******************************************************************************
  * @file    motor_follow.c
  * @brief   位置跟随（leader-follower）：2 号机当"遥控端"，1 号机当"被控端"
  *
  * 一句话做法：**每一拍读一次 leader 的实测位置，把它（加一个起始基准）当成
  * follower 位置环的目标发过去**。闭环在设备里，上位机只负责"开/关"，
  * 不参与每一拍 —— 走上位机中继的话，光一趟链路延迟就是几十 ms 起步，压根跟不上。
  *
  * 为什么"只读位置"就够：
  *   leader 是遥控端，它怎么动都行（我方自动匀速转 / 人手拖 / 失能滑转），
  *   0x74 一查就有位置。所以跟随循环**从不依赖 leader 的控制模式**，
  *   也（除了 spin 自动驱动那条路）不会去动 leader 的状态。
  *
  * 增量式对齐（关键）：
  *     目标 = follower 起始绝对计数 + (leader 当前绝对计数 - leader 起始绝对计数)
  *   两台电机各自的零点都有一段偏置（实测停在 0° 时位置值报 61 而不是 0），
  *   用"增量"就不用标定它：从开机那一刻的角度差开始跟，天然对上。
  *
  * 绝对计数（把"里程"和"位置"拼起来，一次解决两个坑）：
  *     abs = 里程 × 32768 + 位置
  *   位置值 0~32767 是一圈，而且 32767 与 0 是**同一个点**：直接拿位置当角度，
  *   过零点会从 32767 跳回 0（看着像"倒转一圈"），而且想跟多圈也没法表达；
  *   它只有一圈的信息，位置环又是"走最短路径"，一旦一拍转过 180° 就会反向抄近路。
  *   用里程拼成单调递增的绝对角度，这些问题一起消失。
  *
  * 安全（跟随是"电机自己在动"的功能，防护比功能本身重要）：
  *   - 一拍里 leader 位置突变 > 90°（FOLLOW_GLITCH_COUNTS）：电机转不了这么快，
  *     判成丢帧/里程跳变 → 这一拍只重新对齐基准、**不动 follower**
  *     （宁可少跟一下，也不能让它照着假数据猛冲）。
  *   - 跟随误差（follower 落后 leader 多少）超过一圈 → 判失控：急停两台 + 失能 + 断电。
  *   - 连续 FOLLOW_MISS_LIMIT 拍读不到 leader / 写不进 follower → 中止
  *     （"控制不住"比"乱动"安全）。
  *   - 开关跟随只走 MotorCtrl_RemoteCmd 一个入口：OTA 模式会先把跟随停掉，
  *     任何手动命令（失能/急停/走位/切电源）也会先停跟随 —— 否则跟随循环下一拍
  *     就把电机拽回目标位置，急停会"按不住"。
  *
  * ⚠ 不要在这里点 LED：WS2812 走 SPI6，只允许一个任务驱动（现有约定是
  *   MotorCtrl_Task 独占），第二个任务并发点灯会把帧打断。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_follow.h"

#include "motor.h"
#include "motor_ctrl.h"
#include "motor_power.h"    /* MotorPwr_OnAndSettle()：两台电机共用 PC14 那一路电源 */
#include "uart_log.h"

#include "FreeRTOS.h"
#include "task.h"           /* uxTaskGetStackHighWaterMark()：把栈余量打进日志 */
#include "cmsis_os2.h"

#include <stdio.h>

/* Private define ------------------------------------------------------------*/

/* 一圈有多少个计数：位置值 0~32767 是一圈（32767 == 0，同一个点）⇒ 一圈 = 32768 */
#define FOLLOW_COUNTS_PER_TURN    32768

/* leader 一拍最多"合理地"走多少计数：超过 90° 就当假数据（电机最快 2400°/s，
   20 ms 也就 48°，90° 已经放宽一倍了） */
#define FOLLOW_GLITCH_COUNTS      8192

/* spin 斜坡一拍最多往前推多少计数（4096 = 45°）。
   正常转速下跑不到这个数（3600°/s × 20 ms 才 7.2°），但万一哪一拍被拖长了
   （中继中/日志口卡住），也绝不能让目标一次跳半圈 —— 位置环一次跳超过 180°
   会走"较近的那边"，方向都可能反过来。 */
#define FOLLOW_SPIN_MAX_STEP      4096

/* 跟随误差（follower 落后 leader 多少）的两个门槛 */
#define FOLLOW_LAG_WARN_COUNTS    8192     /* 90°：打一行警告，继续跟 */

/* 落后整整一圈：肯定不是"追不上"，是失控（比如 follower 其实不在位置环里）
   ⚠ 别把这个门槛调到"正常跟不上也会触发"的范围，否则转速一给大就自己停了 */
#define FOLLOW_LAG_ABORT_COUNTS   32768

/* 连续几拍读不到 leader / 写不进 follower 就中止 */
#define FOLLOW_MISS_LIMIT         5U

/* 统计窗口 = 日志周期 */
#define FOLLOW_STATS_MS           1000U

/* 日志行缓冲：这个模块的行都短（最长 ~90 字节），统一给 160。
   ⚠ 栈是按嵌套深度叠加的：跟随循环自己的一拍里，MotorFollow_Report 是最深的一层
     （两个行缓冲 + snprintf），而且从 MotorFollow_Abort 进来时还叠着 Step 的帧。
     别把这里的缓冲无谓地放大（见 motor_ctrl.c 里 MOTOR_LINE_SHORT 的注释）。 */
#define FOLLOW_LINE_SIZE          160U

/* 跟随任务栈（字节，CMSIS-RTOS2 的 stack_size 单位是字节）：
   里面有 snprintf（日志行）+ HAL 调用，和轮询任务一样留 2 KB */
#define FOLLOW_TASK_STACK         (512U * 4U)

/* 启动阶段读 leader 的重试：电机刚上电要过一会儿才应答（和使能重试同一个道理） */
#define FOLLOW_READ_RETRY         10U
#define FOLLOW_READ_RETRY_MS      200U

/* Private variables ---------------------------------------------------------*/

static osThreadId_t    s_task = NULL;
static osSemaphoreId_t s_sem  = NULL;

/* 1 = 跟随循环该跑；0 = 该退出。Stop() 先清它、再等 s_busy 落下 */
static volatile uint8_t s_run  = 0U;

/* 1 = 跟随循环正处在"一拍"里（可能正在用总线）。
   Stop() 靠它保证自己发出的命令不会又被跟随循环覆盖掉 */
static volatile uint8_t s_busy = 0U;

static const osThreadAttr_t s_task_attr =
{
    .name       = "motorFollow",
    .stack_size = FOLLOW_TASK_STACK,
    .priority   = (osPriority_t)osPriorityNormal,
};

static uint8_t  s_leader_id   = 0U;   /* 启动时由 MotorCtrl_MotorIdOfIndex() 换算 */
static uint8_t  s_follower_id = 0U;
static uint32_t s_period_ms   = MOTOR_FOLLOW_PERIOD_DEFAULT_MS;

/* ---- leader 自动转动（spin）---- */
static uint32_t s_spin         = 0U;   /* 单位 0.1°/s，0 = 不自动转 */
static uint32_t s_spin_phase   = 0U;   /* 步进的小数累加器（不足 1 个计数的那部分） */
static uint32_t s_spin_last_ms = 0U;   /* 上一次推目标的时间戳（算 dt） */
static int64_t  s_leader_target;       /* 斜坡推到的"绝对目标计数" */

/* ---- 跟随的基准 / 状态 ---- */
static int64_t  s_leader_last   = 0;   /* 上一拍 leader 的绝对计数 */
static int64_t  s_traveled      = 0;   /* leader 累计走了多少计数（相对起始） */
static int64_t  s_follower_base = 0;   /* follower 起始绝对计数 */
static uint8_t  s_have_last     = 0U;  /* 0 = 还没对齐基准（第一拍只对齐，不动作） */
static uint8_t  s_miss_streak   = 0U;  /* 连续没答上的拍数 */
static uint8_t  s_lag_warned    = 0U;  /* 本轮是否已经警告过"落后太多" */
static uint8_t  s_last_leader_fault   = 0U;
static uint8_t  s_last_follower_fault = 0U;

/* ---- 统计 ---- */
static MotorFollowStats s_stats;
static uint32_t s_start_ms   = 0U;     /* 本次跟随开始的时刻 */
static uint32_t s_window_ms  = 0U;     /* 当前统计窗口的起点 */
static uint32_t s_win_ticks  = 0U;     /* 窗口内跑了几拍 */
static int64_t  s_err_sum    = 0;
static int32_t  s_err_max    = 0;
static uint32_t s_err_n      = 0U;

/* Private function prototypes -----------------------------------------------*/
static void    MotorFollow_Task(void *argument);
static void    MotorFollow_Step(void);
static void    MotorFollow_Abort(const char *why);
static void    MotorFollow_Report(void);
static uint8_t MotorFollow_ReadAbs(uint8_t motor_id, int64_t *abs_out,
                                   uint32_t retry, uint32_t retry_ms);

/* Private functions ---------------------------------------------------------*/

/* tick → ms。本工程 configTICK_RATE_HZ = 1000（1 tick = 1 ms），这里仍然按
   osKernelGetTickFreq() 算，免得以后改 tick 率时静默算错 */
static uint32_t MotorFollow_TicksToMs(uint32_t ticks)
{
    uint32_t freq = osKernelGetTickFreq();

    if (freq == 0U)
    {
        freq = 1000U;
    }

    return (uint32_t)(((uint64_t)ticks * 1000ULL) / (uint64_t)freq);
}

/* 绝对计数 → 0.1° 单位（32768 计数 = 一圈 = 3600）。日志不用 %f：
   newlib-nano 默认没开浮点 printf，打出来是空的 */
static int32_t MotorFollow_CountsToDeciDeg(int64_t counts)
{
    return (int32_t)((counts * 3600) / FOLLOW_COUNTS_PER_TURN);
}

/* 把"0.1 为单位"的整数打成 "+12.3" / "-1.2" 这种文本 */
static void MotorFollow_DeciText(int32_t deci, char *out, size_t out_size)
{
    int32_t abs_deci = (deci < 0) ? -deci : deci;

    (void)snprintf(out, out_size, "%s%ld.%ld", (deci < 0) ? "-" : "+",
                   (long)(abs_deci / 10), (long)(abs_deci % 10));
}

/* 把绝对计数折到位置环要的 0..32767（给定值范围） */
static uint16_t MotorFollow_Wrap(int64_t abs_counts)
{
    int64_t m = abs_counts % (int64_t)FOLLOW_COUNTS_PER_TURN;

    if (m < 0)
    {
        m += FOLLOW_COUNTS_PER_TURN;
    }

    return (uint16_t)m;
}

/*
 * 读一台电机的"绝对计数"（里程 × 一圈 + 位置）。
 * retry/retry_ms 用于启动阶段（电机刚上电要过一会儿才应答）。
 * 返回 1 = 成功且 *abs_out 已填好。
 */
static uint8_t MotorFollow_ReadAbs(uint8_t motor_id, int64_t *abs_out,
                                   uint32_t retry, uint32_t retry_ms)
{
    MotorStatus status;

    for (uint32_t attempt = 0U; attempt < retry; attempt++)
    {
        if (Motor_QueryStatus(motor_id, &status) != 0U)
        {
            if (abs_out != NULL)
            {
                *abs_out = ((int64_t)status.mileage * FOLLOW_COUNTS_PER_TURN) +
                           (int64_t)status.position;
            }

            return 1U;
        }

        if (attempt + 1U < retry)
        {
            osDelay(retry_ms);      /* osDelay 的 0 是"不延时"，所以要判一下 */
        }
    }

    return 0U;
}

/*
 * 一拍：leader（可选驱动）→ 读 leader → 写 follower → 读 follower → 统计。
 * ⚠ 函数第一句就是检查 s_run：Stop() 把 s_run 清零后**任何**一拍都必须立刻返回，
 *   不能再去碰总线 —— 否则 Stop() 发的"急停"会被我们后面这一拍覆盖掉。
 */
static void MotorFollow_Step(void)
{
    MotorStatus   st;
    MotorValueAck ack;
    uint32_t      now;
    int64_t       leader_abs;
    int64_t       follower_abs;
    int64_t       delta;
    int64_t       lag;

    if (s_run == 0U)
    {
        return;
    }

    now = osKernelGetTickCount();

    /* ---- 1) leader 自动驱动：自己算目标、一小步一小步推过去 ---- */
    if (s_spin != 0U)
    {
        uint32_t dt_ms = MotorFollow_TicksToMs(now - s_spin_last_ms);

        if (dt_ms == 0U)
        {
            dt_ms = 1U;     /* 同一毫秒里跑了两拍（tick 粒度），按 1 ms 算 */
        }
        s_spin_last_ms = now;

        /* 本拍该走多少计数：
             转速(0.1°/s) → °/s（÷10）→ 计数/s（× 32768 / 360）→ 本拍（× dt/1000）
             = 转速 × dt × 32768 / 3600000
           不足 1 个计数的部分留在 s_spin_phase 里，攒够了下一次补上
           （慢速时才不会"每拍都进位到 0、一直不动"）。
           ⚠ 整个乘积走 uint64：转速上限 36000 × dt 最多 1000 ms 时是 1.2e12，
             直接塞回 uint32_t 会截断，那一拍就会吐一个完全错误的步长。 */
        uint64_t phase = (uint64_t)s_spin_phase +
                         ((uint64_t)s_spin * (uint64_t)dt_ms * 32768ULL);
        int32_t  step  = (int32_t)(phase / 3600000ULL);

        s_spin_phase = (uint32_t)(phase % 3600000ULL);

        if (step > (int32_t)FOLLOW_SPIN_MAX_STEP)
        {
            step = (int32_t)FOLLOW_SPIN_MAX_STEP;
        }

        s_leader_target += step;

        if (Motor_SetValue(s_leader_id, (int16_t)MotorFollow_Wrap(s_leader_target),
                           0U, 0U, &ack) == 0U)
        {
            s_stats.miss_leader++;      /* leader 拖不动了：下一拍读位置也会失败，会走中止 */
        }
    }

    if (s_run == 0U)
    {
        return;
    }

    /* ---- 2) 读 leader 的实测位置（跟随的"源"） ---- */
    if (Motor_QueryStatus(s_leader_id, &st) == 0U)
    {
        s_stats.miss_leader++;
        s_miss_streak++;

        if (s_miss_streak >= FOLLOW_MISS_LIMIT)
        {
            MotorFollow_Abort("leader not answering (0x74)");
        }

        return;
    }

    /* 故障码变了就打一行（电机自己报的问题比我们猜的准） */
    if (st.fault != s_last_leader_fault)
    {
        char line[128];

        s_last_leader_fault = st.fault;
        (void)snprintf(line, sizeof(line),
                       "[follow] leader id%u fault=0x%02X\r\n", (unsigned int)s_leader_id,
                       (unsigned int)st.fault);
        UartLog_Print(line);
    }

    leader_abs = ((int64_t)st.mileage * FOLLOW_COUNTS_PER_TURN) + (int64_t)st.position;

    /* ---- 3) 增量 + 突变保护 ---- */
    if (s_have_last == 0U)
    {
        /* 第一拍只对齐基准（Start 里已经读过一次，这里兜个底） */
        s_leader_last = leader_abs;
        s_have_last   = 1U;
        return;
    }

    delta = leader_abs - s_leader_last;

    if ((delta > FOLLOW_GLITCH_COUNTS) || (delta < -FOLLOW_GLITCH_COUNTS))
    {
        char line[128];

        s_leader_last = leader_abs;     /* 只重新对齐基准，**不动 follower** */
        s_stats.glitch++;

        if (s_stats.glitch <= 3U)
        {
            (void)snprintf(line, sizeof(line),
                           "[follow] leader jumped %ld counts in one step "
                           "(>%u) - skipped, re-based\r\n",
                           (long)delta, (unsigned int)FOLLOW_GLITCH_COUNTS);
            UartLog_Print(line);
        }

        return;
    }

    s_leader_last = leader_abs;
    s_traveled += delta;

    if (s_run == 0U)
    {
        return;
    }

    /* ---- 4) 给 follower 发目标：它的起始位置 + leader 走过的增量 ---- */
    if (Motor_SetValue(s_follower_id,
                       (int16_t)MotorFollow_Wrap(s_follower_base + s_traveled),
                       0U, 0U, &ack) == 0U)
    {
        s_stats.miss_follower++;
        s_miss_streak++;

        if (s_miss_streak >= FOLLOW_MISS_LIMIT)
        {
            MotorFollow_Abort("follower not answering (0x64)");
        }

        return;
    }

    /* ---- 5) 读 follower 的实测位置：算跟随误差 + 防跑飞 ---- */
    if (Motor_QueryStatus(s_follower_id, &st) == 0U)
    {
        s_stats.miss_follower++;
        s_miss_streak++;

        if (s_miss_streak >= FOLLOW_MISS_LIMIT)
        {
            MotorFollow_Abort("follower not answering (0x74)");
        }

        return;
    }

    s_miss_streak = 0U;

    if (st.fault != s_last_follower_fault)
    {
        char line[128];

        s_last_follower_fault = st.fault;
        (void)snprintf(line, sizeof(line),
                       "[follow] follower id%u fault=0x%02X\r\n",
                       (unsigned int)s_follower_id, (unsigned int)st.fault);
        UartLog_Print(line);
    }

    follower_abs = ((int64_t)st.mileage * FOLLOW_COUNTS_PER_TURN) + (int64_t)st.position;

    /* 跟随误差 = 命令它到的地方 - 它实际在的地方（正 = 落在目标后面，没跟上）
       等价于"两台电机的角度差"：leader 走了 s_traveled，follower 只走了
       follower_abs - s_follower_base，差值就是落后的角度 */
    lag = s_traveled - (follower_abs - s_follower_base);

    s_err_sum += lag;
    s_err_n++;

    /* 最大误差按绝对值记（正负都更新）。别写一堆符号判断，两行就够 */
    {
        int64_t lag_abs = (lag < 0) ? -lag : lag;
        int64_t max_abs = (s_err_max < 0) ? -(int64_t)s_err_max : (int64_t)s_err_max;

        if (lag_abs > max_abs)
        {
            s_err_max = (int32_t)lag;
        }
    }

    if ((lag > FOLLOW_LAG_ABORT_COUNTS) || (lag < -FOLLOW_LAG_ABORT_COUNTS))
    {
        char line[128];

        (void)snprintf(line, sizeof(line),
                       "[follow] follower behind %ld counts (limit %u) - runaway?\r\n",
                       (long)lag, (unsigned int)FOLLOW_LAG_ABORT_COUNTS);
        UartLog_Print(line);
        MotorFollow_Abort("following error > 1 turn");
        return;
    }

    if (((lag > FOLLOW_LAG_WARN_COUNTS) || (lag < -FOLLOW_LAG_WARN_COUNTS)) &&
        (s_lag_warned == 0U))
    {
        char line[144];
        char txt[16];

        s_lag_warned = 1U;
        MotorFollow_DeciText(MotorFollow_CountsToDeciDeg(lag), txt, sizeof(txt));
        (void)snprintf(line, sizeof(line),
                       "[follow] WARN: lagging %s deg - too fast for this loop "
                       "(lower the spin or the period)\r\n", txt);
        UartLog_Print(line);
    }

    /* ---- 6) 计时 / 统计窗口 ---- */
    s_stats.ticks++;
    s_win_ticks++;

    if (MotorFollow_TicksToMs(now - s_window_ms) >= FOLLOW_STATS_MS)
    {
        MotorFollow_Report();
    }
}

/*
 * 出事了：立刻把循环停掉、两台电机急停+失能+断电，并把原因打出来。
 * 只能由跟随任务自己调用（它不会去等 s_busy —— 那会儿正握在自己手里）。
 */
static void MotorFollow_Abort(const char *why)
{
    char line[160];

    s_run = 0U;
    s_stats.aborted = 1U;

    (void)snprintf(line, sizeof(line), "[follow] ABORT: %s\r\n", why);
    UartLog_Print(line);

    MotorFollow_Report();       /* 先把最后的数字留下来 */

    MotorCtrl_SafeStopAll();    /* 急停 + 失能两台 + 切 PC14（日志里有它自己几行） */
    MotorCtrl_PollPause(0U);    /* 恢复 200 ms 状态轮询 */
}

/* 把统计打出来（1 Hz，以及 stop/abort 时收尾） */
static void MotorFollow_Report(void)
{
    char     line[FOLLOW_LINE_SIZE];
    char     line2[FOLLOW_LINE_SIZE];
    char     e1[16];
    char     e2[16];
    uint32_t now      = osKernelGetTickCount();
    uint32_t win_ms   = MotorFollow_TicksToMs(now - s_window_ms);
    uint32_t rate_10  = 0U;    /* 频率 ×10（“48.5Hz”要打小数，就不在 printf 里算） */
    int64_t  rev_100  = (s_traveled * 100) / FOLLOW_COUNTS_PER_TURN;   /* 圈数 ×100 */
    uint32_t hwm      = (uint32_t)uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t);

    /* 实测周期：窗口总时长 ÷ 窗口里的拍数 */
    if (s_win_ticks > 0U)
    {
        s_stats.period_us_meas = (uint32_t)(((uint64_t)win_ms * 1000ULL) / s_win_ticks);
    }

    if (s_stats.period_us_meas > 0U)
    {
        rate_10 = 10000000U / s_stats.period_us_meas;
    }

    if (s_err_n > 0U)
    {
        s_stats.err_avg_counts = (int32_t)(s_err_sum / (int64_t)s_err_n);
        s_stats.err_max_counts = s_err_max;
    }

    s_stats.seconds       = MotorFollow_TicksToMs(now - s_start_ms) / 1000U;
    s_stats.travel_counts = (int32_t)s_traveled;
    s_stats.spin          = s_spin;

    MotorFollow_DeciText(MotorFollow_CountsToDeciDeg(s_stats.err_avg_counts), e1, sizeof(e1));
    MotorFollow_DeciText(MotorFollow_CountsToDeciDeg(s_stats.err_max_counts), e2, sizeof(e2));

    /* 两行：一行进度/总线，一行跟随质量（长行拆两行是这边的约定） */
    (void)snprintf(line, sizeof(line),
                   "[follow] t=%lus rate=%lu.%luHz travel=%ld.%02ldrev "
                   "miss=%lu/%lu glitch=%lu overrun=%lu hwm=%lu\r\n",
                   (unsigned long)s_stats.seconds,
                   (unsigned long)(rate_10 / 10U), (unsigned long)(rate_10 % 10U),
                   (long)(rev_100 / 100), (long)(rev_100 % 100),
                   (unsigned long)s_stats.miss_leader, (unsigned long)s_stats.miss_follower,
                   (unsigned long)s_stats.glitch, (unsigned long)s_stats.overrun,
                   (unsigned long)hwm);
    UartLog_Print(line);

    (void)snprintf(line2, sizeof(line2),
                   "[follow]   lag avg=%s max=%s deg (leader id%u -> follower id%u)\r\n",
                   e1, e2, (unsigned int)s_leader_id, (unsigned int)s_follower_id);
    UartLog_Print(line2);

    /* 开新窗口 */
    s_window_ms = now;
    s_win_ticks = 0U;
    s_err_sum   = 0;
    s_err_max   = 0;
    s_err_n     = 0U;
    s_lag_warned = 0U;
}

/*
 * 跟随任务：等信号量 → 一拍一拍跑，直到 s_run 被清掉 → 回去等下一次 Start。
 * 用 osDelayUntil 固定节拍：每拍从"上一拍的起点 + 周期"开始算，
 * 而不是"If(工作) + 延时"，否则实际周期会被每拍的工作时间拖长（实测差很多）。
 */
static void MotorFollow_Task(void *argument)
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
            MotorFollow_Step();
            s_busy = 0U;

            if (s_run == 0U)
            {
                break;
            }

            next += pdMS_TO_TICKS(s_period_ms);

            if ((int32_t)(osKernelGetTickCount() - next) >= 0)
            {
                /* 一拍没跑完就该进下一拍了（总线/电机慢）：记一笔，从"现在"重新排。
                   不记的话会悄悄变成"周期 = 工作时间"，日志里 rate 会骗人 */
                s_stats.overrun++;
                next = osKernelGetTickCount();
            }

            (void)osDelayUntil(next);
        }
    }
}

/* Exported functions --------------------------------------------------------*/

uint8_t MotorFollow_Init(void)
{
    s_leader_id   = MotorCtrl_MotorIdOfIndex(MOTOR_FOLLOW_LEADER_INDEX);
    s_follower_id = MotorCtrl_MotorIdOfIndex(MOTOR_FOLLOW_FOLLOWER_INDEX);

    if ((s_leader_id == 0U) || (s_follower_id == 0U))
    {
        UartLog_Print("[follow] bad motor index config (leader/follower not on the bus)\r\n");
        return 0U;
    }

    if (s_sem == NULL)
    {
        s_sem = osSemaphoreNew(1U, 0U, NULL);
    }

    if (s_task == NULL)
    {
        char line[112];

        s_task = osThreadNew(MotorFollow_Task, NULL, &s_task_attr);

        if (s_task == NULL)
        {
            (void)snprintf(line, sizeof(line),
                           "[follow] task create FAILED (free heap=%lu)\r\n",
                           (unsigned long)xPortGetFreeHeapSize());
            UartLog_Print(line);
            return 0U;
        }
    }

    return ((s_sem != NULL) && (s_task != NULL)) ? 1U : 0U;
}

uint8_t MotorFollow_Start(uint32_t period_ms)
{
    char     line[FOLLOW_LINE_SIZE];
    int64_t  leader_abs   = 0;
    int64_t  follower_abs = 0;
    uint8_t  already      = (s_run != 0U) ? 1U : 0U;
    uint32_t now          = osKernelGetTickCount();
    uint32_t period_req   = period_ms;

    /* 周期：给得比下限还小 = "没给参数"，用默认值，不夹到 5 ms
       （夹下去只会变成"每拍都超时"，日志里 overrun 一直涨，还不如按默认来） */
    if (period_req < MOTOR_FOLLOW_PERIOD_MIN_MS)
    {
        period_req = MOTOR_FOLLOW_PERIOD_DEFAULT_MS;
    }
    else if (period_req > MOTOR_FOLLOW_PERIOD_MAX_MS)
    {
        period_req = MOTOR_FOLLOW_PERIOD_MAX_MS;
    }

    if (already != 0U)
    {
        /* 已经在跟随：只更新周期（让调用方能"调快/调慢"而不用重启） */
        s_period_ms = period_req;
        (void)snprintf(line, sizeof(line),
                       "[follow] already running, period -> %lu ms\r\n",
                       (unsigned long)s_period_ms);
        UartLog_Print(line);
        return 1U;
    }

    if (MotorCtrl_OtaMode() != 0U)
    {
        UartLog_Print("[follow] refused: OTA in progress\r\n");
        return 0U;
    }

    if (MotorFollow_Init() == 0U)
    {
        return 0U;
    }

    s_period_ms = period_req;
    s_spin      = 0U;       /* 每次开始都从"不自动转"起（要自动转再发 spin） */
    s_spin_phase = 0U;

    UartLog_Print("[follow] start: enable + position loop, then track leader position\r\n");

    /* 两台电机共用 PC14 那一路电源：先上电、等稳定，再谈命令 */
    MotorPwr_OnAndSettle();

    /* follower 必须在位置环：使能（自带重试）→ 0xA0/0x03 → 0x75 复核模式值 */
    if (MotorCtrl_PreparePositionLoop(s_follower_id) == 0U)
    {
        UartLog_Print("[follow] start FAILED: follower is not in position loop (0x03)\r\n");
        MotorCtrl_SafeStopAll();
        return 0U;
    }

    /* 基准：两台各自的绝对计数。leader 这里只是"读"，它是什么模式都不影响 */
    if (MotorFollow_ReadAbs(s_leader_id, &leader_abs,
                            FOLLOW_READ_RETRY, FOLLOW_READ_RETRY_MS) == 0U)
    {
        UartLog_Print("[follow] start FAILED: leader not answering (0x74)\r\n");
        MotorCtrl_SafeStopAll();
        return 0U;
    }

    if (MotorFollow_ReadAbs(s_follower_id, &follower_abs, 3U, 100U) == 0U)
    {
        UartLog_Print("[follow] start FAILED: follower not answering (0x74)\r\n");
        MotorCtrl_SafeStopAll();
        return 0U;
    }

    s_leader_last      = leader_abs;
    s_leader_target    = leader_abs;      /* spin 的斜坡从"现在的位置"起跑 */
    s_follower_base    = follower_abs;
    s_traveled         = 0;
    s_have_last        = 1U;
    s_miss_streak      = 0U;
    s_lag_warned       = 0U;
    s_last_leader_fault   = 0U;
    s_last_follower_fault = 0U;
    s_spin_last_ms     = now;

    /* 统计清零 */
    s_stats.running        = 1U;
    s_stats.aborted        = 0U;
    s_stats.leader_id      = s_leader_id;
    s_stats.follower_id    = s_follower_id;
    s_stats.period_ms      = s_period_ms;
    s_stats.period_us_meas = 0U;
    s_stats.ticks          = 0U;
    s_stats.seconds        = 0U;
    s_stats.spin           = 0U;
    s_stats.travel_counts  = 0;
    s_stats.err_avg_counts = 0;
    s_stats.err_max_counts = 0;
    s_stats.miss_leader    = 0U;
    s_stats.miss_follower  = 0U;
    s_stats.glitch         = 0U;
    s_stats.overrun        = 0U;

    s_start_ms  = now;
    s_window_ms = now;
    s_win_ticks = 0U;
    s_err_sum   = 0;
    s_err_max   = 0;
    s_err_n     = 0U;

    /* 200 ms 状态轮询要停掉：它也在这条总线上，跟着一起抢只会让两边都变慢 */
    MotorCtrl_PollPause(1U);

    s_run = 1U;
    (void)osSemaphoreRelease(s_sem);

    (void)snprintf(line, sizeof(line),
                   "[follow] follower id%u -> position loop OK, leader id%u base=%ld, "
                   "period=%lu ms\r\n",
                   (unsigned int)s_follower_id,
                   (unsigned int)s_leader_id, (long)leader_abs,
                   (unsigned long)s_period_ms);
    UartLog_Print(line);
    UartLog_Print("[follow]   numbers every 1 s; `ctrl spin <0.1deg/s>` to auto-spin"
                  " the leader, `ctrl follow 0` to stop\r\n");

    return 1U;
}

uint8_t MotorFollow_Stop(void)
{
    char line[FOLLOW_LINE_SIZE];

    if (s_run == 0U)
    {
        return 1U;      /* 本来就没在跑：当成已经是停的 */
    }

    s_run = 0U;
    s_spin = 0U;

    /* 等当前这一拍跑完。s_run 已经清零 ⇒ 它下一拍开头就会直接返回、不再碰总线，
       所以我们后面发的命令不会被它覆盖（Step 的第一句就是查 s_run）。 */
    if (osThreadGetId() != s_task)
    {
        uint32_t guard = 0U;

        while ((s_busy != 0U) && (guard < 100U))
        {
            osDelay(1U);
            guard++;
        }
    }

    MotorCtrl_PollPause(0U);

    s_stats.running = 0U;
    MotorFollow_Report();       /* 收尾打一次总账 */

    (void)snprintf(line, sizeof(line), "[follow] stop: t=%lus ticks=%lu\r\n",
                   (unsigned long)s_stats.seconds, (unsigned long)s_stats.ticks);
    UartLog_Print(line);

    return 1U;
}

uint8_t MotorFollow_IsRunning(void)
{
    return (s_run != 0U) ? 1U : 0U;
}

uint8_t MotorFollow_SetSpin(uint32_t tenth_degps)
{
    char    line[FOLLOW_LINE_SIZE];
    int64_t leader_abs = 0;
    uint8_t mode;

    if (tenth_degps > MOTOR_FOLLOW_SPIN_MAX)
    {
        tenth_degps = MOTOR_FOLLOW_SPIN_MAX;
    }

    if (tenth_degps == 0U)
    {
        /* 停 = 斜坡不再往前推，目标冻结 ⇒ 位置环让它就地停住 */
        if (s_spin != 0U)
        {
            s_spin = 0U;
            UartLog_Print("[follow] spin stopped (target frozen, leader holds)\r\n");
        }

        return 1U;
    }

    if (s_run == 0U)
    {
        UartLog_Print("[follow] spin refused: start following first (ctrl follow)\r\n");
        return 0U;
    }

    if (MotorFollow_Init() == 0U)
    {
        return 0U;
    }

    /* leader 得先在位置环里（我们靠给它"目标位置"来匀速转它）。
       已经是 0x03 就别再折腾一遍（使能重试 + 切模式要 1 s 上下）。 */
    mode = MotorCtrl_QueryModeValue(s_leader_id);

    if (mode != MOTOR_MODE_POSITION)
    {
        UartLog_Print("[follow] spin: leader is not in position loop, setting it up ...\r\n");

        if (MotorCtrl_PreparePositionLoop(s_leader_id) == 0U)
        {
            UartLog_Print("[follow] spin FAILED: leader is not in position loop (0x03)\r\n");
            return 0U;
        }
    }

    if (MotorFollow_ReadAbs(s_leader_id, &leader_abs, 3U, 100U) == 0U)
    {
        UartLog_Print("[follow] spin FAILED: leader not answering (0x74)\r\n");
        return 0U;
    }

    /* 先把 leader 按在当前位置上，再起跑。
       ⚠ 必须这么做：切进位置环之后，"给定值"寄存器里可能还是上一次留下的数字
         （比如以前测试时发的 8191），不先按一下它就会朝那个老目标猛冲。 */
    {
        MotorValueAck ack;

        (void)Motor_SetValue(s_leader_id, (int16_t)MotorFollow_Wrap(leader_abs),
                             0U, 0U, &ack);
    }

    s_leader_target = leader_abs;
    s_leader_last   = leader_abs;      /* 基准也对齐，免得下一拍算出一个假的巨量增量 */
    s_have_last     = 1U;
    s_spin_phase    = 0U;
    s_spin_last_ms  = osKernelGetTickCount();
    s_spin          = tenth_degps;

    (void)snprintf(line, sizeof(line),
                   "[follow] spin: leader id%u -> %lu.%lu deg/s (position-loop ramp), "
                   "target=%ld\r\n",
                   (unsigned int)s_leader_id,
                   (unsigned long)(tenth_degps / 10U), (unsigned long)(tenth_degps % 10U),
                   (long)leader_abs);
    UartLog_Print(line);

    return 1U;
}

uint32_t MotorFollow_GetSpin(void)
{
    return s_spin;
}

void MotorFollow_GetStats(MotorFollowStats *out)
{
    if (out == NULL)
    {
        return;
    }

    *out = s_stats;
    out->running = (s_run != 0U) ? 1U : 0U;
    out->spin    = s_spin;
}
