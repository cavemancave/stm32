/**
  ******************************************************************************
  * @file    motor_ctrl.c
  * @brief   电机业务流程：按键使能 → 切位置环 → 0 点 / 3 点来回 → 再按键失能，
  *          外加状态轮询
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
#include "motor_follow.h"   /* 位置跟随（2 号机 → 1 号机）：跟随循环在它自己的任务里 */
#include "motor_power.h"    /* 电机电源开关（PC14） */
#include "uart_log.h"
#include "ws2812.h"

/* 无线控制命令的状态码（OTA_OK / OTA_E_xxx）和这个入口的约定在这里 */
#include "ota_layout.h"
#include "ota_trace.h"

#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include <stdio.h>

/* 轮询定时器（在 Core/Src/freertos.c 里创建），升级期间要把它停下来 */
extern osTimerId_t motorPosHandle;

/* Private define ------------------------------------------------------------*/

/* 挂在同一条串口上的电机台数（和下面 motor_ids[] 的元素个数要一致）
   现在 2 台：ID1 + ID2，共用同一条 USART10 单总线、也共用 PC14 那一路电机电源。
   ⚠ 达妙这颗的 ID 是**上电时锁存**的（ID 脚 低 = ID1 / 高 = ID2），所以一条总线最多两台。 */
#define MOTOR_COUNT                 2U

/* USER_KEY（PA15）消抖时间：按下后隔这么久再确认一次 */
#define KEY_DEBOUNCE_MS             20U

/* 按键按下的电平：按键接地 → 按下读到低电平。
   如果实测反了（不按也一直触发），把这里改成 GPIO_PIN_SET */
#define USER_KEY_PRESSED_LEVEL      GPIO_PIN_RESET

/* 「边走边等按键」时的检查间隔：把动作之间的停顿切成这么长的小段，
   按键按下最多晚这么多时间被发现（10ms 够跟手了） */
#define KEY_POLL_MS                 10U

/* 日志行缓冲大小（调用方栈上，别放在这个文件里当全局，省得抢）。
   256 是算出来的：最长的状态行（含 position 的 0.1°、故障码文本和 10 字节 raw）
   最坏情况约 168 字节，留足余量；改短了 gcc 会报 -Wformat-truncation。 */
#define MOTOR_LINE_SIZE             256U

/* 短行缓冲：只给“肯定短”（< 140 字节）且**处在深调用链里**的日志用。
   ⚠ 栈是按嵌套深度叠加的：MOTOR_LINE_SIZE 的行缓冲如果和另一个嵌套的函数
     （比如 MotorCtrl_LogMode）同时活着，光这两个就要 700 字节栈。
     实测踩过：跟随功能把无线命令那条链压深了 ~550 字节，otaSvc 任务的
     2 KB 栈被压爆，而症状是 FreeRTOS 里一个毫不相干的断言（栈溢出把堆里
     挨着的队列控制块写坏了），查了很久。能省就省。 */
#define MOTOR_LINE_SHORT            144U

/* 10 字节 raw 打成文本的最长长度：2*10 + 9 个空格 + '\0' */
#define MOTOR_HEX_SIZE              32U

/* ---- 位置环往返 ---- */

/* 0 点（12 点方向）= 0°，3 点方向 = 90°；位置值 0~32767 对应 0~360° */
#define MOTOR_TARGET_A_DEG          0U
#define MOTOR_TARGET_B_DEG          90U

/* 0x64 的 DATA[6] 加速时间：速度环下每 1rpm 用多少 ms；位置环填 0（按 1 处理） */
#define MOTOR_MOVE_ACCEL_TIME       0x00U

/* 0x64 的 DATA[7] 刹车：0xFF 刹车，其它值不刹车（速度环下有效） */
#define MOTOR_MOVE_BRAKE            0x00U

/* 每走到一个目标点停多久再往回走 */
#define MOTOR_MOVE_DWELL_MS         1000U

/* ---- 位置环动作：单次测试 还是 0°↔90° 往返 ---- */

/* 1 = 单次测试：切进位置环后只“回起点 + 分步走满一圈”就停，每一步都把
         “命令的位置 -> 实际停的位置”、误差和里程圈数打出来；
   0 = 原来的 0 点 <-> 3 点 来回走。 */
#define MOTOR_SINGLE_MOVE_TEST      1U

/* ⚠ 先搞清楚刻度：位置值 0~32767 对应 0~360°，也就是**一圈**，而且 32767 与 0 是同一个点。
   所以直接命令 32767 等于“回到 0 点”：电机本来就停在 0° 附近，位置环按最短路径算，
   它压根不用动（实测：命令 32767 后 position 一直停在 61/65，里程 0 -> 0）。
   要走满一圈必须**分几步**走，否则永远看不到它转。
   这三个都是**位置值**（不是角度）：0° = 0、90° = 8191、360° = 32767。 */
#define MOTOR_TEST_START_POS        0U       /* 起点：0° */
#define MOTOR_TEST_STEP_POS         8191U    /* 每步 90°（8191 = 90°） */
#define MOTOR_TEST_STEPS            4U       /* 4 步 = 一整圈 */

/* 单次测试：一步走完之后**不是固定等一个时间**，而是“等到它不动了”。
   位置环接近目标时会爬很久（实测：等 1.5 s 时还差 8~13°，而且还在动 —— 每 200 ms 还在走
   200~700 个计数），固定等一个时间会把它误判成“没走够”。
   下面这几个是“停住”的判定条件和最长等待：
     - 刚发完命令先等 MIN_WAIT，别把“还没启动”当成“已经停住”；
     - 每隔 POLL 读一次 0x74，位置单次变化 <= STILL_COUNTS（约 0.11°）算“没动”，
       连续 STILL_SAMPLES 次都这样就算停稳了；
     - 最多等 SETTLE_MAX，超时就把当前值当结果（停顿期间按键可随时打断）。 */
#define MOTOR_TEST_SETTLE_MAX_MS    8000U
#define MOTOR_TEST_MIN_WAIT_MS      600U
#define MOTOR_TEST_POLL_MS          200U
#define MOTOR_TEST_STILL_COUNTS     10U
#define MOTOR_TEST_STILL_SAMPLES    3U

/* ---- 位置环自检 / 防跑飞 ---- */

/* 位置环下走一步（最多 270°）里程最多变 ±1 圈。
   里程变化超过这个值 = 给定的 "位置" 其实被当成了电流/转速，电机在连轴转 ——
   实测踩过这个坑：目标 90°，电机 1 秒连转 3 圈不停（每 200ms 查一次，
   位置 25770/13020/372/20282/7629 乱跳，里程 9→12）。
   宁可误停（丢帧，不会误判）也不要让电机一直转。 */
#define MOTOR_MAX_TURNS_PER_MOVE    1

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
   跟随模块（motor_follow.c）也用它把“2 号机”换成帧首字节，所以是公开的。 */
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

/* 指示灯下一次要用枚举里的哪个颜色（上电第一个状态 = 红）。
   ⚠ LED 只由 MotorCtrl_Task 一个任务驱动：WS2812 走 SPI6，两个任务同时点灯
     会把帧打断。轮询任务只打日志，不碰灯。 */
static WS2812_Color_t s_led_color = WS2812_COLOR_RED;

/* 1 = 无线 OTA 升级中：电机强制失能、停轮询、禁止再命令走动。
   由 OtaService 在会话开始/结束时置位/清除（见 MotorCtrl_SetOtaMode） */
static uint8_t s_ota_mode = 0U;

/* 200 ms 轮询被暂停的次数（不是布尔值）：OTA 会话和位置跟随各会停一次，
   计数归零才真的把定时器开回来 —— 用布尔值的话，先恢复的那一个
   会把另一个还需要的“暂停”也一并解开，于是轮询和跟随一起抢总线 */
static uint8_t s_poll_pause_cnt = 0U;

/* Private function prototypes -----------------------------------------------*/
static void           MotorCtrl_PollTask(void *argument);
static void           MotorCtrl_PollOnce(void);
static uint8_t        MotorCtrl_KeyPressed(void);
static void           MotorCtrl_WaitKeyPress(void);
static uint8_t        MotorCtrl_DelayOrKeyPress(uint32_t ms);
static WS2812_Color_t MotorCtrl_LedNextColor(void);
static void           MotorCtrl_HexToText(const uint8_t *data, uint8_t len,
                                          char *out, size_t out_size);
static uint8_t        MotorCtrl_SetModeRetry(uint8_t motor_id, uint8_t mode,
                                             MotorMode *ack, uint32_t retry_ms,
                                             uint32_t retry_max);
static uint8_t        MotorCtrl_Enable(uint8_t motor_id, MotorMode *ack);
static uint8_t        MotorCtrl_Disable(uint8_t motor_id, MotorMode *ack);
static void           MotorCtrl_DisableOne(uint8_t motor_id);
static void           MotorCtrl_DisableAll(void);
static uint8_t        MotorCtrl_LogMode(uint8_t motor_id);
static uint8_t        MotorCtrl_ReadStatus(uint8_t motor_id, int32_t *mileage, uint16_t *position);
static uint8_t        MotorCtrl_TurnsRunaway(uint8_t motor_id, int32_t turns_before);
static void           MotorCtrl_Stop(uint8_t motor_id);
#if MOTOR_SINGLE_MOVE_TEST
static int32_t        MotorCtrl_AngleDiff(int32_t expected, int32_t measured);
static uint8_t        MotorCtrl_WaitStill(uint8_t motor_id);
static void           MotorCtrl_LogMoveResult(uint8_t motor_id, const char *tag, uint16_t target,
                                              int32_t turns_before, uint8_t before_ok,
                                              uint16_t pos_before);
static uint8_t        MotorCtrl_TestStep(uint8_t motor_id, uint16_t target, const char *tag);
#endif
static void           MotorCtrl_LogVersion(uint8_t motor_id);
static uint8_t        MotorCtrl_EnterPositionLoop(uint8_t motor_id, MotorMode *ack);
static uint8_t        MotorCtrl_EnterPositionLoopAll(void);
static uint8_t        MotorCtrl_MoveToPosition(uint8_t motor_id, uint16_t position, const char *tag);
#if !MOTOR_SINGLE_MOVE_TEST
static uint8_t        MotorCtrl_MoveTo(uint8_t motor_id, uint16_t angle_deg);   /* 只有 0°↔90° 往返用得到 */
#endif

/* Private functions ---------------------------------------------------------*/

/*
 * 每次点灯都调它取枚举里的下一个颜色：
 * RED → GREEN → BLUE → YELLOW → CYAN → MAGENTA → WHITE → RED ...
 * 跳过 WS2812_COLOR_OFF（灭）和 WS2812_COLOR_COUNT（只是个计数）。
 */
static WS2812_Color_t MotorCtrl_LedNextColor(void)
{
    WS2812_Color_t color = s_led_color;

    s_led_color = (WS2812_Color_t)((uint32_t)s_led_color + 1U);
    if ((uint32_t)s_led_color >= (uint32_t)WS2812_COLOR_COUNT)
    {
        s_led_color = WS2812_COLOR_RED;   /* 绕回第一个颜色 */
    }

    return color;
}

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
 * 查一次按键（USER_KEY = PA15，低电平按下），**不等按键**：
 * 有「按下 + 消抖后仍然按下」才算一次，然后等松手，这样按住不放也只算一次触发。
 * 返回 1 = 这一次查到按键，0 = 当前没按。
 * 原来 main.c 里用的是 HAL_Delay，搬到任务里必须换成 osDelay（会让出 CPU）。
 */
static uint8_t MotorCtrl_KeyPressed(void)
{
    if (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) != USER_KEY_PRESSED_LEVEL)
    {
        return 0U;   /* 没按下：立刻返回，不耽误当前动作 */
    }

    osDelay(KEY_DEBOUNCE_MS);   /* 消抖：过一会儿再看还是不是按下 */

    if (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) != USER_KEY_PRESSED_LEVEL)
    {
        return 0U;   /* 消抖后已经不按了 → 当成抖动丢掉 */
    }

    /* 等松手：不然按住不放会在循环里被当成连按 */
    while (HAL_GPIO_ReadPin(USER_KEY_GPIO_Port, USER_KEY_Pin) == USER_KEY_PRESSED_LEVEL)
    {
        osDelay(KEY_POLL_MS);
    }

    return 1U;
}

/*
 * 等一次按键：一直查到有一次按下为止（按住不放只算一次，语义和以前一样）。
 */
static void MotorCtrl_WaitKeyPress(void)
{
    while (MotorCtrl_KeyPressed() == 0U)
    {
        osDelay(KEY_POLL_MS);
    }
}

/*
 * 停顿 ms 毫秒，停顿期间顺便看有没有按键（把 ms 切成 KEY_POLL_MS 的小段）。
 * 电机正在来回走的时候用它代替 osDelay：这样「再按一下键」不用等停顿结束才被看见。
 * 返回 1 = 停到一半按了键（调用方该中止当前动作），0 = 停满 ms 没按键。
 */
static uint8_t MotorCtrl_DelayOrKeyPress(uint32_t ms)
{
    for (uint32_t waited = 0U; waited < ms; waited += KEY_POLL_MS)
    {
        /* 升级中（otaSvc 起了会话）：当成"被打断"，让主流程赶紧退回等按键，
           别再往总线上发动作命令 */
        if (s_ota_mode != 0U)
        {
            return 1U;
        }

        if (MotorCtrl_KeyPressed() != 0U)
        {
            return 1U;
        }

        osDelay(KEY_POLL_MS);
    }

    return 0U;
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

/*
 * 查一次模式（0x75 -> 0x76）并打一行日志。
 * ⚠ 0x75 既是「其他反馈」的反馈码，又是「模式查询」的发送码，靠上下文区分。
 * 返回电机报的模式值；查不到（超时/不答）返回 MOTOR_MODE_UNKNOWN。
 */
static uint8_t MotorCtrl_LogMode(uint8_t motor_id)
{
    MotorMode mode;
    char      line[MOTOR_LINE_SIZE];
    char      hex[MOTOR_HEX_SIZE];
    uint8_t   ok = Motor_QueryMode(motor_id, &mode);

    MotorCtrl_HexToText(mode.raw, MOTOR_FRAME_SIZE, hex, sizeof(hex));

    (void)snprintf(line, sizeof(line),
                   "id=%u mode query (0x75): reply=%s, mode=0x%02X (%s), rx=%s\r\n",
                   (unsigned int)motor_id, (ok != 0U) ? "OK" : "NONE",
                   (unsigned int)mode.mode, Motor_ModeName(mode.mode), hex);
    UartLog_Print(line);

    return (ok != 0U) ? mode.mode : (uint8_t)MOTOR_MODE_UNKNOWN;
}

/*
 * 读一次 0x74 的里程 / 位置（防跑飞和单次测试都要用）。
 * mileage / position 可以传 NULL（不关心）。返回 1 = 成功，失败时两个输出都不动。
 */
static uint8_t MotorCtrl_ReadStatus(uint8_t motor_id, int32_t *mileage, uint16_t *position)
{
    MotorStatus status;

    if (Motor_QueryStatus(motor_id, &status) == 0U)
    {
        return 0U;
    }

    if (mileage != NULL)
    {
        *mileage = status.mileage;
    }

    if (position != NULL)
    {
        *position = status.position;
    }

    return 1U;
}

/*
 * 防跑飞：和 MOTOR_MAX_TURNS_PER_MOVE 比较，超过就报警。
 * 里程读不到时返回 0（不报警）：偶尔丢一帧不该把正常工作停下来。
 */
static uint8_t MotorCtrl_TurnsRunaway(uint8_t motor_id, int32_t turns_before)
{
    int32_t turns_now = 0;
    int32_t delta;

    if (MotorCtrl_ReadStatus(motor_id, &turns_now, NULL) == 0U)
    {
        return 0U;
    }

    delta = turns_now - turns_before;

    if (delta < 0)
    {
        delta = -delta;
    }

    if (delta > MOTOR_MAX_TURNS_PER_MOVE)
    {
        char line[MOTOR_LINE_SIZE];

        (void)snprintf(line, sizeof(line),
                       "id=%u RUNAWAY: mileage %ld -> %ld (%ld turns) while moving <=270deg\r\n",
                       (unsigned int)motor_id, (long)turns_before, (long)turns_now,
                       (long)delta);
        UartLog_Print(line);

        return 1U;
    }

    return 0U;
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

#if MOTOR_SINGLE_MOVE_TEST
/*
 * 位置差按“最短路径”算：位置值一圈是 0~32767，而且 32767 和 0 是同一个点，
 * 所以差要折到 -16384..+16384（正 = 停过头了，负 = 没走够）。
 */
static int32_t MotorCtrl_AngleDiff(int32_t expected, int32_t measured)
{
    int32_t diff = measured - expected;

    if (diff > 16384)
    {
        diff -= 32768;
    }
    else if (diff < -16384)
    {
        diff += 32768;
    }

    return diff;
}

/*
 * 等这一步停下来：每 MOTOR_TEST_POLL_MS 读一次位置，连续 MOTOR_TEST_STILL_SAMPLES 次
 * 变化都不超过 MOTOR_TEST_STILL_COUNTS 才算停稳；最多等 MOTOR_TEST_SETTLE_MAX_MS。
 * 返回 1 = 等的时候按了键（后面自然会走“按键 = 失能”那条路）。
 */
static uint8_t MotorCtrl_WaitStill(uint8_t motor_id)
{
    uint16_t last      = 0;
    uint8_t  have_last = 0U;
    uint8_t  still     = 0U;
    uint32_t waited    = 0U;

    while (waited < MOTOR_TEST_SETTLE_MAX_MS)
    {
        uint16_t now = 0;

        if (MotorCtrl_DelayOrKeyPress(MOTOR_TEST_POLL_MS) != 0U)
        {
            return 1U;
        }

        waited += MOTOR_TEST_POLL_MS;

        if (waited < MOTOR_TEST_MIN_WAIT_MS)
        {
            continue;   /* 刚发完命令、电机还没启动，先别判“停住” */
        }

        if (MotorCtrl_ReadStatus(motor_id, NULL, &now) == 0U)
        {
            continue;   /* 丢一帧不算，接着等 */
        }

        if (have_last != 0U)
        {
            int32_t moved = MotorCtrl_AngleDiff((int32_t)last, (int32_t)now);

            if (moved < 0)
            {
                moved = -moved;
            }

            still = (moved <= (int32_t)MOTOR_TEST_STILL_COUNTS) ? (uint8_t)(still + 1U) : 0U;
        }

        last      = now;
        have_last = 1U;

        if (still >= MOTOR_TEST_STILL_SAMPLES)
        {
            break;      /* 停稳了 */
        }
    }

    return 0U;
}

/*
 * 单次测试一步的收尾：再读一次 0x74，打一行
 *   "single move 2/4 -> 180.0deg: position 61 (0.6deg) -> 16444 (180.7deg),
 *    error=+61 (+0.7deg), mileage 0 -> 0 (0 turns)"
 * error = 实际停的位置 - 命令的位置（最短路径），它就是“这一步到底走了多少度”的答案：
 *   命令走 90° 却只走了 45°，error 就是 -4096（-45.0°）；
 *   每步 error 都差不多（一个固定的值）= 零点偏移，说明刻度是对的。
 * 里程差 = 实际转了几圈（只有跨过 0°/360° 那个点才会 +1）。
 */
static void MotorCtrl_LogMoveResult(uint8_t motor_id, const char *tag, uint16_t target,
                                    int32_t turns_before, uint8_t before_ok,
                                    uint16_t pos_before)
{
    MotorStatus status;
    char        line[MOTOR_LINE_SIZE];
    uint16_t    deci_after;
    uint16_t    deci_before;
    int32_t     error;
    int32_t     err_deci;
    unsigned long err_abs;

    if (Motor_QueryStatus(motor_id, &status) == 0U)
    {
        (void)snprintf(line, sizeof(line), "id=%u %s: status query (0x74) FAILED (no 0x75)\r\n",
                       (unsigned int)motor_id, tag);
        UartLog_Print(line);
        return;
    }

    deci_after  = Motor_PositionToDeciDeg(status.position);
    deci_before = Motor_PositionToDeciDeg(pos_before);

    error    = MotorCtrl_AngleDiff((int32_t)target, (int32_t)status.position);
    err_deci = (error * 3600) / 32768;                      /* 误差换算成 0.1° */
    err_abs  = (unsigned long)((err_deci < 0) ? -err_deci : err_deci);

    if (before_ok != 0U)
    {
        (void)snprintf(line, sizeof(line),
                       "id=%u %s: position %u (%u.%udeg) -> %u (%u.%udeg), error=%ld (%s%lu.%ludeg), "
                       "mileage %ld -> %ld (%ld turns)\r\n",
                       (unsigned int)motor_id,
                       tag, (unsigned int)pos_before,
                       (unsigned int)(deci_before / 10U), (unsigned int)(deci_before % 10U),
                       (unsigned int)status.position,
                       (unsigned int)(deci_after / 10U), (unsigned int)(deci_after % 10U),
                       (long)error, (err_deci < 0) ? "-" : "+",
                       err_abs / 10UL, err_abs % 10UL,
                       (long)turns_before, (long)status.mileage,
                       (long)(status.mileage - turns_before));
    }
    else
    {
        (void)snprintf(line, sizeof(line),
                       "id=%u %s: position -> %u (%u.%udeg), error=%ld (%s%lu.%ludeg), "
                       "mileage=%ld (no baseline)\r\n",
                       (unsigned int)motor_id,
                       tag, (unsigned int)status.position,
                       (unsigned int)(deci_after / 10U), (unsigned int)(deci_after % 10U),
                       (long)error, (err_deci < 0) ? "-" : "+",
                       err_abs / 10UL, err_abs % 10UL, (long)status.mileage);
    }

    UartLog_Print(line);
}

/*
 * 单次测试走一步：读里程+位置 → 发 0x64 → 等到位置不再变（可按键打断）→ 打结果。
 * 返回 1 = 这一步的里程变化超过 MOTOR_MAX_TURNS_PER_MOVE，说明电机没在位置环里，
 * 调用方应该急停（和来回走时用的是同一个防跑飞阀值）。
 */
static uint8_t MotorCtrl_TestStep(uint8_t motor_id, uint16_t target, const char *tag)
{
    int32_t  turns_before = 0;
    uint16_t pos_before   = 0;
    uint8_t  before_ok    = MotorCtrl_ReadStatus(motor_id, &turns_before, &pos_before);

    (void)MotorCtrl_MoveToPosition(motor_id, target, tag);
    WS2812_SetColor(MotorCtrl_LedNextColor());

    if (MotorCtrl_WaitStill(motor_id) != 0U)
    {
        UartLog_Print("key pressed during test step\r\n");
    }

    MotorCtrl_LogMoveResult(motor_id, tag, target, turns_before, before_ok, pos_before);

    return ((before_ok != 0U) && (MotorCtrl_TurnsRunaway(motor_id, turns_before) != 0U)) ? 1U : 0U;
}
#endif /* MOTOR_SINGLE_MOVE_TEST */

/* 查一次版本号（0xFD -> 0xFE）。失败也把收到的原始字节打出来，方便定位 */
static void MotorCtrl_LogVersion(uint8_t motor_id)
{
    MotorVersion version;
    char         line[MOTOR_LINE_SIZE];
    char         hex[MOTOR_HEX_SIZE];

    uint8_t ok = Motor_QueryVersion(motor_id, &version);

    MotorCtrl_HexToText(version.raw, MOTOR_FRAME_SIZE, hex, sizeof(hex));

    (void)snprintf(line, sizeof(line),
                   "id=%u version query (0xFD): reply=%s, rx=%s\r\n",
                   (unsigned int)motor_id, (ok != 0U) ? "OK" : "NONE", hex);
    UartLog_Print(line);

    if (ok != 0U)
    {
        /* 年字节是 20XX 的 XX（2021 年 = 0x15 = 十进制 21），所以按十进制打 */
        (void)snprintf(line, sizeof(line),
                       "id=%u version: date=20%02u-%02u-%02u, model=0x%02X, fw=0x%02X, hw=0x%02X\r\n",
                       (unsigned int)motor_id,
                       (unsigned int)version.year, (unsigned int)version.month,
                       (unsigned int)version.day, (unsigned int)version.model,
                       (unsigned int)version.fw_version, (unsigned int)version.hw_version);
        UartLog_Print(line);
    }
}

/*
 * 切位置环：发 0xA0，模式值 MOTOR_MODE_POSITION(0x03)，回帧的模式值从 *ack 带出来。
 * ⚠ 手册 5.3 的模式表里**没有**位置环，0x03 来自厂家上位机工程
 *   （docs/M0603A 系列直驱电机.bit 的模式值下拉框里就有「位置环 = 03」）
 *   和 docs/M63A快捷指令集.cfg：「电机1切换位置环=1|0|16|01A003000000000000D9」。
 *
 * 返回 1 **只表示“收到了一条合法的 0xA1 回帧”，不代表电机真的进了位置环**：
 * 不认 0x03 的固件也会回一个合法的 0xA1，只是模式值不是 0x03。
 * 所以按主流程的约定，由调用方（MotorCtrl_Task）在这之后再用 0x75/0x76 查一次模式复核。
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
       ⚠ 这里**刻意不用 MotorCtrl_LogMode()**：它自带 256 字节行缓冲，和本函数
         的行缓冲嵌套着同时活着 —— 光这一对就要 700 字节栈，而本函数很可能是
         从无线命令（otaSvc 任务）这条路进来的（实测就把那个任务的栈压爆过，
         见 MOTOR_LINE_SHORT 的注释）。就地查一次模式值、用一行短的打出来。 */
    mode_before = MotorCtrl_QueryModeValue(motor_id);

    (void)snprintf(line, sizeof(line),
                   "id=%u mode before switch (expect 0x01 current loop): 0x%02X (%s)\r\n",
                   (unsigned int)motor_id, (unsigned int)mode_before,
                   Motor_ModeName(mode_before));
    UartLog_Print(line);

    uint8_t ok = Motor_SetMode(motor_id, MOTOR_MODE_POSITION, ack);

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

#if !MOTOR_SINGLE_MOVE_TEST
/*
 * 位置环下走一个角度：0° -> 0、90° -> 8191，换算走 Motor_AngleToPosition()。
 * （只有 0°↔90° 往返用得到；单次测试用的是下面的 MotorCtrl_MoveToPosition()）
 */
static uint8_t MotorCtrl_MoveTo(uint8_t motor_id, uint16_t angle_deg)
{
    char tag[24];

    (void)snprintf(tag, sizeof(tag), "move to %udeg", (unsigned int)angle_deg);

    return MotorCtrl_MoveToPosition(motor_id, Motor_AngleToPosition(angle_deg), tag);
}
#endif /* !MOTOR_SINGLE_MOVE_TEST */

/*
 * 把所有电机都切到位置环并**逐一复核**（切完必须用 0x75 再查一次，只看 0xA1 不够：
 * 不认 0x03 的固件也会回一个合法的 0xA1，那时电机其实还在电流/速度环里）。
 * 返回 1 = 每一台都确认在位置环；0 = 有哪台没切上/没答上（调用方负责失能）。
 */
static uint8_t MotorCtrl_EnterPositionLoopAll(void)
{
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        MotorMode mode_ack;

        if (MotorCtrl_EnterPositionLoop(motor_ids[i], &mode_ack) == 0U)
        {
            UartLog_Print("set position loop FAILED (no valid 0xA1)\r\n");
            return 0U;
        }

        /* 切完**再查一次模式**（0x75 -> 0x76），看看到底设置成功了没。
           只看 0xA1「有没有回帧」是不够的：不认 0x03 的固件也会回一个合法的 0xA1，
           这时电机其实还在电流环/速度环里，后面按“位置”发 0x64（8191）会被当成
           电流/转速，电机就一直转（实测：目标 90°，1 秒连转 3 圈不停）。 */
        UartLog_Print("mode after switch (expect 0x03 position loop):\r\n");

        uint8_t mode_after = MotorCtrl_LogMode(motor_ids[i]);

        if (mode_after == MOTOR_MODE_UNKNOWN)
        {
            /* 模式查询没答上（老固件？）：退回看 0xA1 echo 里的模式值 */
            UartLog_Print("mode query did not answer, falling back to the 0xA1 echo\r\n");
            mode_after = mode_ack.mode;
        }

        if (mode_after != MOTOR_MODE_POSITION)
        {
            char line[MOTOR_LINE_SIZE];

            (void)snprintf(line, sizeof(line),
                           "id=%u position loop NOT active (mode=0x%02X, %s): 0x64 would be "
                           "read as current/speed - not sweeping\r\n",
                           (unsigned int)motor_ids[i], (unsigned int)mode_after,
                           Motor_ModeName(mode_after));
            UartLog_Print(line);
            return 0U;
        }
    }

    UartLog_Print("position loop confirmed (0x03) on all motors\r\n");

    return 1U;
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
 * 最后把电机电源（PC14）也关掉：既然可控电源就是用来"不用的时候不带电"的，
 * 失能完还给它送电就没意义了（而且升级/意外时电机带电更不安全）。
 * 下次要使能会先重新上电（MotorPwr_OnAndSettle）。
 *
 * ⚠ 只有"全部失能"才切电源：两台共用同一路电源（PC14），单独失能某一台时
 *   另一台可能还在干活，不能把它的电也断了（单台失能走 MotorCtrl_DisableOne）。
 */
static void MotorCtrl_DisableAll(void)
{
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        MotorCtrl_DisableOne(motor_ids[i]);
    }

    MotorPwr_Enable(0U);     /* 把电机电源也切了（没变化时不会重复写） */
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
 *   进入 = 电机全部失能 + 停 200 ms 轮询 + 禁止后续走动命令
 *   退出 = 恢复轮询（电机仍然保持失能，要动就再按键/发 ENABLE）
 */
void MotorCtrl_SetOtaMode(uint8_t on)
{
    if ((on != 0U) && (s_ota_mode == 0U))
    {
        s_ota_mode = 1U;

        MotorCtrl_PollPause(1U);                      /* 先停轮询，别和 OTA 抢带宽 */

        /* 再停跟随：它自己会去改电机状态、而且占着总线。
           （它内部的 PollPause(0) 把上面那次暂停退回去，计数仍是 1 = 暂停。） */
        (void)MotorFollow_Stop();

        MotorCtrl_DisableAll();                       /* 最后失能：升级期间电机必须不使劲 */
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
                    1..MOTOR_COUNT = 指定那一台（= motor_ids[] 里的下标 +1） */
    uint8_t   motor_id = MotorCtrl_MotorIdOfIndex((motor_index == 0U) ? 1U : motor_index);

    if ((motor_index > MOTOR_COUNT))
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
       电源也只准"关"（OTA_BEGIN 自己会先断电），不许在升级期间把电机重新上电。
       （位置跟随的两个命令不在名单里 ⇒ 升级中发过来会被下面这段挡掉） */
    if ((s_ota_mode != 0U) && (cmd != OTA_CTRL_DISABLE) && (cmd != OTA_CTRL_STOP) &&
        (cmd != OTA_CTRL_LOG_MUTE) && (cmd != OTA_CTRL_POLL_PAUSE) &&
        !((cmd == OTA_CTRL_PWR) && (arg == 0U)))
    {
        return OTA_E_NOTALLOWED;
    }

    /* 跟随在跑的时候：作用于 **follower（1 号机）或“全部”** 的手动命令要先停跟随，
       否则跟随循环下一拍就把电机拽回目标位置，急停 / 失能会“按不住”；
       而**指名 leader（2 号机）** 的命令不停跟随 —— 那正是“自己驱动遥控端”那条路子。 */
    if ((MotorFollow_IsRunning() != 0U) &&
        (cmd != OTA_CTRL_FOLLOW) && (cmd != OTA_CTRL_SPIN) &&
        ((motor_index == 0U) || (motor_index == MOTOR_FOLLOW_FOLLOWER_INDEX)))
    {
        UartLog_Print("motor: follow stopped by a manual command\r\n");
        (void)MotorFollow_Stop();
    }

    switch (cmd)
    {
        case OTA_CTRL_DISABLE:
            /* 不指定电机（老上位机）= 全部失能，连电源一起切；
               指定了 = 只失能那一台，**不切电源**（另一台还要干活） */
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
            /* 用可嵌套的暂停接口（别直接 osTimerStop）：跟随/OTA 也在用同一个定时器 */
            MotorCtrl_PollPause((arg != 0U) ? 1U : 0U);
            break;

        case OTA_CTRL_FOLLOW:
            /* 位置跟随（2 号机当遥控端 → 1 号机跟着转）：arg = 0 停 / 其它 = 周期 ms
               回复：data = 1 在跑、data2 = 实测周期 µs、data3 = 平均跟随误差（计数） */
            if (arg == 0U)
            {
                (void)MotorFollow_Stop();
            }
            else if (MotorFollow_Start(arg) == 0U)
            {
                ok = 0U;
            }

            {
                MotorFollowStats fs;

                MotorFollow_GetStats(&fs);

                if (out  != NULL) { *out  = MotorFollow_IsRunning(); }
                if (out2 != NULL) { *out2 = fs.period_us_meas; }
                if (out3 != NULL) { *out3 = (uint32_t)fs.err_avg_counts; }
            }
            break;

        case OTA_CTRL_SPIN:
            /* leader 自动匀速转：arg = 转速（0.1°/s，0 = 停）。
               回复：data = 生效转速、data2 = leader 的总线 ID */
            if (MotorFollow_SetSpin(arg) == 0U)
            {
                ok = 0U;
            }

            if (out != NULL)
            {
                *out = MotorFollow_GetSpin();
            }

            if (out2 != NULL)
            {
                *out2 = MotorCtrl_MotorIdOfIndex(MOTOR_FOLLOW_LEADER_INDEX);
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

            if (out != NULL)
            {
                *out = MotorPwr_IsOn();    /* 告诉上位机现在到底是开是关 */
            }
            break;

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

/* ---- 给跟随模块（motor_follow.c）用的内部工具 -----------------------------
 * 这些活（使能 / 切位置环 / 急停 / 停轮询）在本文件里都已经实现过一遍了，
 * 直接复用，免得跟随模块再写一套、以后两边各自漂移。 */

/*
 * 查一次模式值（0x75 -> 0x76），**不打日志**：
 * 跟随循环里"要不要重新配位置环"这种判断不能每次都往日志里灌。
 * 查不到（超时/不答）返回 MOTOR_MODE_UNKNOWN。
 */
uint8_t MotorCtrl_QueryModeValue(uint8_t motor_id)
{
    MotorMode mode;

    return (Motor_QueryMode(motor_id, &mode) != 0U) ? mode.mode : (uint8_t)MOTOR_MODE_UNKNOWN;
}

/*
 * 把一台电机弄进位置环：使能（自带重试，最多约 5 s）→ 0xA0/0x03 → 0x75 复核。
 * 返回 1 = 确认已在位置环（0x03）；0 = 没成功（原因在日志里）。
 *
 * ⚠ 只看 0xA1"有没有回帧"是不够的：不认 0x03 的固件也会回一个合法的 0xA1，
 *   那时电机其实还在电流/速度环里，我们再按"位置"发 0x64 就会被当成电流给定
 *   （实测踩过：目标 90°，电机 1 秒连转 3 圈）。
 */
uint8_t MotorCtrl_PreparePositionLoop(uint8_t motor_id)
{
    MotorMode ack;
    uint8_t   mode_after;

    if (MotorCtrl_Enable(motor_id, &ack) == 0U)
    {
        UartLog_Print("motor: enable FAILED (no valid 0xA1) - check power/wiring\r\n");
        return 0U;
    }

    if (MotorCtrl_EnterPositionLoop(motor_id, &ack) == 0U)
    {
        return 0U;      /* 日志已经在那边打过了 */
    }

    mode_after = MotorCtrl_LogMode(motor_id);       /* 0x75 -> 0x76 复核模式值 */

    if (mode_after != MOTOR_MODE_POSITION)
    {
        char line[MOTOR_LINE_SHORT];

        (void)snprintf(line, sizeof(line),
                       "id=%u position loop NOT active (mode=0x%02X, %s) - refusing\r\n",
                       (unsigned int)motor_id, (unsigned int)mode_after,
                       Motor_ModeName(mode_after));
        UartLog_Print(line);
        return 0U;
    }

    UartLog_Print("position loop confirmed (0x03)\r\n");

    return 1U;
}

/* 急停两台（0x64 给定值 0）→ 失能两台 → 切 PC14 电源。给"出事就停"用 */
void MotorCtrl_SafeStopAll(void)
{
    for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
    {
        MotorCtrl_Stop(motor_ids[i]);
    }

    MotorCtrl_DisableAll();
}

/*
 * 暂停 / 恢复 200 ms 状态轮询。**用计数而不是布尔**：
 * OTA 会话和位置跟随都要把它停掉，两边都恢复了才该重新开始 ——
 * 布尔值的话，先恢复的那一个会把另一个还需要的"暂停"一并解开，
 * 于是轮询和跟随又一起抢那条总线。
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

void MotorCtrl_Task(void *argument)
{
    char line[MOTOR_LINE_SIZE];

    (void)argument;

    /* 启动诊断：能打到这里说明调度器起来了、任务在跑。
       如果这个 J 出来了但下面 UartLog_Print 的正文没出来 → 问题在日志口（USART1）。 */
    OtaTrace_Text("[app] J: defaultTask (MotorCtrl_Task) running\r\n");

    UartLog_Print("motor control (FreeRTOS): USART10 38400, press USER_KEY (PA15) to start\r\n");

    WS2812_SetColor(MotorCtrl_LedNextColor());   /* 第一个状态 = 红，等按键 */

    for (;;)
    {
        MotorCtrl_WaitKeyPress();

        /* 跟随正在跑的时候，按键流程会和跟随循环抢总线（而且它会切电源、改模式、
           还会让电机自己走一圈）—— 直接忽略这一次按键，让用户先发
           `ctrl follow 0` 把跟随停掉。灯也不换，免得看起来像"按键生效了"。 */
        if (MotorFollow_IsRunning() != 0U)
        {
            UartLog_Print("key ignored: position follow is running (ctrl follow 0 to stop)\r\n");
            continue;
        }

        WS2812_SetColor(MotorCtrl_LedNextColor());   /* 按键已按下：换下一个颜色 */

        UartLog_Print("key pressed, enabling motor (0xA0/0x08, retry until answered)...\r\n");

        /* 先把电机电源（PC14）拉起来、等电源轨稳定再发使能帧。
           ⚠ 上电后驱动板要复位、电机内部要启动，头几条命令本来就会被丢掉；
           不等这一下，下面那个 25 次重试就是拿来找这个"开机不响应"的。 */
        MotorPwr_OnAndSettle();

        uint8_t enabled_ok = 0U;

        for (uint8_t i = 0U; i < MOTOR_COUNT; i++)
        {
            MotorMode ack;
            char      hex[MOTOR_HEX_SIZE];

            if (MotorCtrl_Enable(motor_ids[i], &ack) == 0U)
            {
                MotorCtrl_HexToText(ack.raw, MOTOR_FRAME_SIZE, hex, sizeof(hex));

                (void)snprintf(line, sizeof(line),
                               "id=%u enable FAILED (no valid 0xA1), rx=%s. press again.\r\n",
                               (unsigned int)motor_ids[i], hex);
                UartLog_Print(line);
                continue;
            }

            /* 不管成没成，都把 RX 收到的 10 字节原样打出来：
               全 0 = 一个字节都没收到（没接线/没上电/波特率不对） */
            MotorCtrl_HexToText(ack.raw, MOTOR_FRAME_SIZE, hex, sizeof(hex));

            (void)snprintf(line, sizeof(line),
                           "id=%u enable OK (0xA1), mode=0x%02X (%s), rx=%s\r\n",
                           (unsigned int)motor_ids[i], (unsigned int)ack.mode,
                           Motor_ModeName(ack.mode), hex);
            UartLog_Print(line);

            MotorCtrl_LogVersion(motor_ids[i]);

            enabled_ok++;
        }

        /* 有一台没使能上就退回等按键，别硬往下切位置环 */
        if (enabled_ok != MOTOR_COUNT)
        {
            UartLog_Print("enable FAILED, press USER_KEY to retry\r\n");
            continue;
        }

        /* 使能成功之后不再换色：接下来由「每走一步换一个颜色」当心跳 */

        /* 两台都切到位置环并逐一复核；有一台没切上就整组失能，不硬往下走 */
        if (MotorCtrl_EnterPositionLoopAll() == 0U)
        {
            MotorCtrl_DisableAll();
            WS2812_SetColor(WS2812_COLOR_OFF);
            UartLog_Print("motor disabled, press USER_KEY to enable again\r\n");
            continue;   /* 重新等按键 */
        }

        /* ⚠ 台架测试（下面那段“走一圈”）**只跑 1 号机**，故意不两台同时走：
           总线上同时发 0x64 会让两台一起转，桌面上容易撞，日志也会混在一起。
           2 号机要用的时候发无线命令：
             ctrl posloop  --motor 2   → 切位置环
             ctrl movepos 8191 --motor 2   → 走 90°
             ctrl movepos 0 --motor 2      → 回 0°
             ctrl disable --motor 2        → 单台失能 */

        uint8_t runaway = 0U;

#if MOTOR_SINGLE_MOVE_TEST
        /* ---- 单次测试：先回起点，再分 4 步（每步 90°）走满一圈 ----
           ⚠ 0~32767 是**一圈**，而且 32767 ≡ 0：直接命令 32767 等于“回 0 点”，
             电机本来就在 0° 附近，位置环按最短路径算，它压根不用动
             （实测：命令 32767 后 position 一直停在 61/65，里程 0 -> 0，看着像“没转”）。
             所以要走满一圈必须分步走，每步看它到底走了多少。
           每步走完停 MOTOR_TEST_SETTLE_MS 再读 0x74，打一行
             "single move 2/4 -> 180.0deg: position 61 (0.6deg) -> 16444 (180.7deg),
              error=+61 (+0.7deg), mileage 0 -> 0 (0 turns)"
           error = 实际停的位置 - 命令的位置（最短路径）：命令走 90° 却只走了 45°，
           error 就是 -4096（-45.0°）。4 步走完里程应该 +1（= 一整圈）。
           要恢复 0°↔90° 往返，把 MOTOR_SINGLE_MOVE_TEST 改成 0U。 */
        int32_t turns_zero = 0;
        uint8_t zero_ok;

        /* ⚠ 台架测试只跑 1 号机（见上面那段说明）：这里写死 motor_ids[0] */
        runaway = MotorCtrl_TestStep(motor_ids[0], MOTOR_TEST_START_POS, "single move 0 -> 0.0deg");

        /* 记下“回 0° 之后”的里程当基准：下面 4 步加起来应该正好 1 圈 */
        zero_ok = MotorCtrl_ReadStatus(motor_ids[0], &turns_zero, NULL);

        for (uint32_t step = 1U; (step <= MOTOR_TEST_STEPS) && (runaway == 0U); step++)
        {
            uint16_t target = (uint16_t)(MOTOR_TEST_STEP_POS * step);
            uint16_t deci   = Motor_PositionToDeciDeg(target);
            char     tag[40];

            (void)snprintf(tag, sizeof(tag), "single move %lu/%u -> %u.%ludeg",
                           (unsigned long)step, (unsigned int)MOTOR_TEST_STEPS,
                           (unsigned int)(deci / 10U), (unsigned long)(deci % 10U));

            runaway = MotorCtrl_TestStep(motor_ids[0], target, tag);
        }

        if ((runaway == 0U) && (zero_ok != 0U))
        {
            int32_t turns_end = 0;

            if (MotorCtrl_ReadStatus(motor_ids[0], &turns_end, NULL) != 0U)
            {
                (void)snprintf(line, sizeof(line),
                               "single-move test done: %u x 90deg should be 1 turn -> mileage "
                               "%ld -> %ld (%ld turns)\r\n",
                               (unsigned int)MOTOR_TEST_STEPS, (long)turns_zero,
                               (long)turns_end, (long)(turns_end - turns_zero));
                UartLog_Print(line);
            }
        }

        UartLog_Print("press USER_KEY to disable, again to re-run\r\n");
#else
        /* 0 点 <-> 3 点 来回走；每走一步换一个灯色，顺便当心跳。
           停顿期间一直查按键（MotorCtrl_DelayOrKeyPress）：再按一下键 = 失能，
           按下去最多 KEY_POLL_MS 就跳出这个循环，不用等当前这一步走完。
           每一步都用里程对一次（MotorCtrl_TurnsRunaway）：只让走 90° 却转了好几圈，
           说明电机根本没在位置环里，立刻急停 + 失能，不让它继续转。 */
        while (runaway == 0U)
        {
            int32_t turns_before = 0;
            uint8_t baseline_ok  = MotorCtrl_ReadMileage(motor_ids[0], &turns_before);

            (void)MotorCtrl_MoveTo(motor_ids[0], MOTOR_TARGET_A_DEG);
            WS2812_SetColor(MotorCtrl_LedNextColor());

            if (MotorCtrl_DelayOrKeyPress(MOTOR_MOVE_DWELL_MS) != 0U)
            {
                break;
            }

            if ((baseline_ok != 0U) && (MotorCtrl_TurnsRunaway(motor_ids[0], turns_before) != 0U))
            {
                runaway = 1U;
                break;
            }

            baseline_ok = MotorCtrl_ReadMileage(motor_ids[0], &turns_before);

            (void)MotorCtrl_MoveTo(motor_ids[0], MOTOR_TARGET_B_DEG);
            WS2812_SetColor(MotorCtrl_LedNextColor());

            if (MotorCtrl_DelayOrKeyPress(MOTOR_MOVE_DWELL_MS) != 0U)
            {
                break;
            }

            if ((baseline_ok != 0U) && (MotorCtrl_TurnsRunaway(turns_before) != 0U))
            {
                runaway = 1U;
                break;
            }
        }
#endif /* MOTOR_SINGLE_MOVE_TEST */

        if (runaway != 0U)
        {
            /* 电机没在位置环里（0x64 被当成电流/转速给了）：先停、再查模式、最后失能 */
            MotorCtrl_Stop(motor_ids[0]);
            (void)MotorCtrl_LogMode(motor_ids[0]);
            MotorCtrl_DisableAll();
            WS2812_SetColor(WS2812_COLOR_OFF);

            UartLog_Print("motor stopped & disabled, press USER_KEY to retry\r\n");
            continue;
        }

        /* ---- 第二次按键 = 失能 ---- */
        UartLog_Print("key pressed again, disabling motor (0xA0/0x09)...\r\n");

        MotorCtrl_DisableAll();

        /* 灯灭 = 电机已失能；再按一次键又会使能（并接着换下一个颜色） */
        WS2812_SetColor(WS2812_COLOR_OFF);

        UartLog_Print("motor disabled, press USER_KEY to enable again\r\n");
    }
}
