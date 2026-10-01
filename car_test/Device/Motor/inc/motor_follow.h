#ifndef __MOTOR_FOLLOW_H__
#define __MOTOR_FOLLOW_H__

#include "main.h"

/*
 * 位置跟随（leader-follower / 遥操作从动）
 *
 * 需求：**2 号机 = 遥控端（自动转或手拖），1 号机 = 被控端（尽量实时跟着转）**。
 *
 * 做法（详细设计见 motor_follow.c 的文件头注释）：
 *   一拍 = ①（可选）驱动 leader 自动匀速转 → ② 读 leader 实测位置（0x74）
 *          → ③ 把位置（起始基准 + 增量）发给 follower 的位置环（0x64）
 *          → ④ 读 follower 实测位置，算跟随误差、防跑飞
 *   闭环全在设备里，上位机只发"开/关"，不参与每一拍。
 *
 * ⚠ 跟随循环只读 leader 的**位置**，所以 leader 是"位置环匀速转"还是"失能被手拖"
 *   都一样能跟 —— 只有 spin（自动驱动）那条路会去改 leader 的模式。
 *
 * ⚠ 周期不是"想要多快就多快"：两台电机挂在**同一条 38400 单总线**上，一问一答，
 *   一个来回约 3 ms；一拍要跑 4 条帧（见上），所以最快也就 50~150 Hz。
 *   日志里的 rate=xx.xHz 就是实测值，要调周期请拿那个数说话。
 */

/* 跟随的两端：序号 = motor_ctrl.c 里 motor_ids[] 的下标 + 1，也就是"几号机" */
#define MOTOR_FOLLOW_LEADER_INDEX     2U     /* 遥控端 */
#define MOTOR_FOLLOW_FOLLOWER_INDEX   1U     /* 被控端 */

/* 跟随周期（ms）。默认 25 ms：一拍要跑 4 条帧（驱动 leader + 读 leader + 写 follower
   + 读 follower），**实测一拍 24 ms 上下**（≈42 Hz）—— 周期给到 20 ms 反而会"每拍都
   算超时"（日志里 overrun 会一直涨，那个计数就没意义了）。
   `ctrl follow <n>` 的 n 小于下限时按**默认值**处理（= 没给参数），大于上限就夹到上限。 */
#define MOTOR_FOLLOW_PERIOD_DEFAULT_MS  25U
#define MOTOR_FOLLOW_PERIOD_MIN_MS      5U
#define MOTOR_FOLLOW_PERIOD_MAX_MS      200U

/* leader 自动转速的上限（单位 0.1°/s）：36000 = 3600°/s。
   ⚠⚠ **但实测这个电机在位置环里的速度上限只有 ~45°/s**（≈450）：
     2026-09-20 实测：命令 90°/s 时它只跑到 42°/s（是一直顶在最大速度上），
     而且之前 4×90° 那轮里每步 ~1.7~2 s = 45~53°/s —— 同一个上限。
     顶在最大速度上 = 反作用力矩最大，**直驱台架会被拖着动**，所以建议给 30°/s（300）以内；
     这个宏只是防手滑，不要拿它当“能力”。 */
#define MOTOR_FOLLOW_SPIN_MAX           36000U

/* ---- 速度前馈（“提前量”）----------------------------------------------
   位置环在**匀速运动**下有一个正比于速度的稳态跟随误差（实测 42°/s 时 22°），
   把它折成时间就是等效延迟：22° ÷ 42°/s = **约 0.5 s**。也就是它的“刚度”
   Kp ≈ 2 /s：要让电机跑某速度，目标就得领先它 速度×0.5 s 那一点。
   所以给目标加一个提前量就能把这个稳态误差基本压掉：
     目标 = 实测位置 + 速度 × 提前量
   - 提前量 = 0 = 纯跟随（原来的行为，会稳定落后一截）；
   - 提前量取到实测的那个等效延迟（~500 ms）时，稳态滞后基本为 0；
   - 再大就会过冲/抖（相当于把相位提前过头），要调就小幅调。
   ⚠ 它只能抵消**稳态**（匀速）那一部分；速度突变时的瞬态滞后消不掉 —— 那是位置环带宽。 */
#define MOTOR_FOLLOW_LEAD_DEFAULT_MS    500U
#define MOTOR_FOLLOW_LEAD_MAX_MS        2000U

/* 提前量本身的上限（计数，8192 = 90°）：防止手拖一下高频拉出个巨大的目标跳变 */
#define MOTOR_FOLLOW_LEAD_MAX_COUNTS    8192

/* 统计快照：1 Hz 的日志和无线命令的回帧都用它 */
typedef struct
{
    uint8_t  running;          /* 1 = 跟随循环在跑 */
    uint8_t  aborted;          /* 1 = 上一次是被保护逻辑掐掉的（不是正常 stop） */
    uint8_t  leader_id;        /* leader 的总线 ID（帧首字节，不是"几号机"） */
    uint8_t  follower_id;      /* follower 的总线 ID */
    uint32_t period_ms;        /* 配置的周期 */
    uint32_t period_us_meas;   /* 实测平均周期（µs）；还没测够一个窗口时为 0 */
    uint32_t ticks;            /* 本次一共跑了几拍 */
    uint32_t seconds;          /* 本次一共跑了多少秒 */
    uint32_t spin;             /* leader 当前自动转速（0.1°/s），0 = 没在自动转 */
    uint32_t lead_ms;          /* 速度前馈提前量（ms），0 = 纯跟随 */
    int32_t  travel_counts;    /* leader 累计走过的计数（32768 = 一圈） */
    int32_t  err_avg_counts;   /* 最近一个统计窗口的平均跟随误差（计数，带符号） */
    int32_t  err_max_counts;   /* 最近一个统计窗口的最大跟随误差（按绝对值取，带符号） */
    uint32_t miss_leader;      /* leader 没答上的累计次数 */
    uint32_t miss_follower;    /* follower 没答上的累计次数 */
    uint32_t glitch;           /* leader 位置突变、这一拍被丢弃的累计次数 */
    uint32_t overrun;          /* 一拍没在周期内跑完的累计次数 */
} MotorFollowStats;

/* 建跟随任务（懒创建：第一次 Start 时建）。返回 1 = 任务已就绪。
   任务栈从 FreeRTOS 堆里出（2 KB），失败会打一行带 free heap 的日志。 */
uint8_t MotorFollow_Init(void);

/* 开始跟随。
   period_ms：跟随周期，< 下限按默认 20 ms、> 上限夹到上限。
   会做的事：电机上电并等稳定 → follower 使能 + 切位置环 0x03 + 用 0x75 复核
             → 记下两台的起始绝对位置 → 暂停 200 ms 状态轮询 → 起循环。
   返回 1 = 已开始；0 = 没起来（原因在日志里：follower 进不了位置环 / leader 不答）。 */
uint8_t MotorFollow_Start(uint32_t period_ms);

/* 停止跟随（等当前这一拍跑完，最多约一个周期）。返回 1 = 确实停了。
   注意：leader 的自动转动也一起停（目标冻结在最后那一拍，位置环让它就地停住）。 */
uint8_t MotorFollow_Stop(void);

/* 现在在跟随吗 */
uint8_t MotorFollow_IsRunning(void);

/* 让 leader（2 号机）自动匀速转，单位 **0.1°/s**（0 = 停）。
   ⚠ 只能在跟随运行中调用（"驱动器"是跟随循环里那个斜坡），没在跟随返回 OTA_E_STATE。
   ⚠ 会在需要时把 leader 使能 + 切位置环，并先把它**按在当前位置**上再起跑，
     所以它不会因为"切模式瞬间目标为 0"而猛冲。
   返回值见下面的状态码约定：1 = 成功，0 = 参数/状态不对。 */
uint8_t MotorFollow_SetSpin(uint32_t tenth_degps);

/* 当前 leader 自动转速（0.1°/s），0 = 没在自动转 */
uint32_t MotorFollow_GetSpin(void);

/* 速度前馈提前量（ms）。0 = 纯跟随；默认见 MOTOR_FOLLOW_LEAD_DEFAULT_MS。
   只改一个数，不用重启跟随。返回生效后的值。 */
uint32_t MotorFollow_SetLead(uint32_t ms);
uint32_t MotorFollow_GetLead(void);

/* 取统计快照（out 可以为 NULL）。原子性不保证"整块一致"，只是给日志/回帧看个大概。 */
void MotorFollow_GetStats(MotorFollowStats *out);

#endif /* __MOTOR_FOLLOW_H__ */
