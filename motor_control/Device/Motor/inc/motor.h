#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "main.h"

/*
 * 串口电机驱动（一问一答，固定 10 字节帧）
 * 手册: docs/M0603A_111_motor_specification_cn.pdf（协议摘录: docs/specification.md）
 *
 * 物理连接: USART10，PE2 = USART10_RX、PE3 = USART10_TX，38400 8N1
 *          （配置在 usart.c / MX_USART10_UART_Init 里，不要只改这里的波特率）
 *
 * ⚠ PE3 (USART10_TX) 必须配成**开漏** (GPIO_MODE_AF_OD)，不能是推挽 (AF_PP)：
 *    电机控制板是 **5V TTL** 电平的单总线（LIN 那种一问一答的接法，TX/RX 挂在同一根线上），
 *    高电平靠总线上的上拉电阻拉到 5V。TX 用推挽的话，MCU 会在线上主动输出 3.3V 高电平，
 *    输出级的 MOS 管一直导通：轻则和 5V 上拉互相灌电流、电平打不上去，
 *    重则把总线拉出错误的电平，被电机当成一帧数据 → **误发信号/总线冲突**。
 *    开漏之后 MCU 只负责"拉低"，"释放"时由外部上拉决定高电平，就不会乱发。
 *    配置在 CubeMX 里对应：PE3 的 GPIO Settings → Output type = **Open Drain**
 *    （只改 usart.c 里的代码没用，下次 Generate Code 会被覆盖）。
 *
 * 总线:     可以挂多台电机，靠帧首字节的 ID 区分（ID 脚低电平 = ID1、高电平 = ID2，
 *          上电时锁存）；一次只问一台，等回复了再问下一台。
 *
 * 帧格式:   DATA[0] | DATA[1] | DATA[2..8] | DATA[9]  —— ID | 功能码 | 数据 | CRC8
 *
 *   ┌──────────┬────────┬────────┬────────────────────────────────────────────────┐
 *   │ 功能      │ 发送码 │ 反馈码 │ DATA[2..8]                                     │
 *   ├──────────┼────────┼────────┼────────────────────────────────────────────────┤
 *   │ 驱动转动  │ 0x64   │ 0x65   │ 速度H 速度L 电流H 电流L 加速时间 温度 故障码      │
 *   │ 其他反馈  │ 0x74   │ 0x75   │ 里程×4 位置H 位置L 故障码                       │
 *   │ 模式切换  │ 0xA0   │ 0xA1   │ 模式值 0 0 0 0 0 0                             │
 *   │ 模式查询  │ 0x75   │ 0x76   │ 模式值 0 0 0 0 0 0                             │
 *   │ 版本查询  │ 0xFD   │ 0xFE   │ 年 月 日 电机型号 软件版本 硬件版本 保留          │
 *   └──────────┴────────┴────────┴────────────────────────────────────────────────┘
 *   ⚠ 0x75 既是"其他反馈"的反馈码，又是"模式查询"的发送码，按上下文区分。
 *
 * 版本号: 年省略 20XX 的 20（2021 年 = 0x15），11 月 = 0x0B，28 号 = 0x1C，型号 63 = 0x3F
 *
 * CRC8: 手册"CRC8 值"一节写明算法为 CRC-8/MAXIM，
 *       多项式 x^8+x^5+x^4+1 (0x31)，输入/输出都反射，初值 0x00，不异或；
 *       反射后右移实现用的多项式是 0x8C（= 0x31 位反转），见 MOTOR_CRC8_POLY。
 *
 * 上电流程（手册"操作步骤"）: ①配置 ID ②发使能指令（使能后默认电流环）③发给定值。
 *
 * ⚠ 使能（0xA0/0x08）的反馈不是把 0x08 回显回来，而是回**使能之后的实际模式**：
 *   实测 ID1 使能后收到 01 A1 01 00 00 00 00 00 00 E0 —— 模式值 0x01 = 默认的电流环。
 *   所以判成功只看"有没有收到合法的 0xA1 回帧"（ID/功能码/CRC 都对），
 *   不要写成 `mode == 0x08`，那样永远不成功。
 *   电机刚上电后要过一段时间才应答，要重试的话由调用方自己循环（见 main.c）。
 */

/* 固定 10 字节帧 */
#define MOTOR_FRAME_SIZE         10U

/* DATA[2..8] 的字节数 */
#define MOTOR_FRAME_PAYLOAD_SIZE (MOTOR_FRAME_SIZE - 3U)

/* 功能码：发送 */
#define MOTOR_CMD_SET_VALUE      0x64U  /* 驱动电机转动 */
#define MOTOR_CMD_GET_STATUS     0x74U  /* 获取其他反馈（里程/位置/故障码） */
#define MOTOR_CMD_GET_MODE       0x75U  /* 获取模式反馈 */
#define MOTOR_CMD_SET_MODE       0xA0U  /* 模式切换 / 使能 / 失能 */
#define MOTOR_CMD_VERSION_QUERY  0xFDU  /* 获取版本号 */

/* 功能码：反馈 */
#define MOTOR_REPLY_SET_VALUE    0x65U
#define MOTOR_REPLY_GET_STATUS   0x75U
#define MOTOR_REPLY_GET_MODE     0x76U
#define MOTOR_REPLY_SET_MODE     0xA1U
#define MOTOR_REPLY_VERSION      0xFEU

/* 模式值（0xA0 发送帧的 DATA[2]；反馈帧 DATA[2] 里带的是切换后的实际模式，
   比如发 0x08 使能之后回的是 0x01 电流环，不会回显 0x08） */
#define MOTOR_MODE_OPEN_LOOP     0x00U  /* 开环 */
#define MOTOR_MODE_CURRENT       0x01U  /* 电流环 */
#define MOTOR_MODE_SPEED         0x02U  /* 速度环 */
#define MOTOR_MODE_ENABLE        0x08U  /* 电机使能（使能后默认电流环） */
#define MOTOR_MODE_DISABLE       0x09U  /* 电机失能 */
#define MOTOR_MODE_TURN_BACK_150 0x0AU  /* 电机后转 150±10° */
#define MOTOR_MODE_LINK_LOSS_ON  0x10U  /* 开启通讯断联功能（>3S 没消息则停止动作） */
#define MOTOR_MODE_LINK_LOSS_OFF 0x11U  /* 关闭通讯断联功能 */

/* 故障码位（0x74 反馈的 DATA[8]） */
#define MOTOR_FAULT_HALL         (1U << 0)  /* 霍尔故障 */
#define MOTOR_FAULT_OVERCURRENT  (1U << 1)  /* 过流故障 */
#define MOTOR_FAULT_STALL        (1U << 3)  /* 堵转故障 */
#define MOTOR_FAULT_OVERTEMP     (1U << 4)  /* 过温故障 */
#define MOTOR_FAULT_LINK_LOSS    (1U << 5)  /* 断联故障 */
#define MOTOR_FAULT_VOLTAGE      (1U << 6)  /* 过欠压故障 */

/* 一帧收/发超时：10 字节 @38400 只要 ~2.6ms，留足余量即可 */
#define MOTOR_FRAME_TIMEOUT_MS   100U

/* CRC-8/MAXIM 反射（右移）形式的多项式：0x31 位反转 = 0x8C */
#define MOTOR_CRC8_POLY          0x8CU

/* 版本号反馈 */
typedef struct
{
  uint8_t id;         /* 反馈帧的 ID（= 查询时用的 ID） */
  uint8_t year;       /* 年（20XX 的 XX） */
  uint8_t month;      /* 月 1-12 */
  uint8_t day;        /* 日 1-31 */
  uint8_t model;      /* 电机型号，63 = 0x3F */
  uint8_t fw_version; /* 软件版本 */
  uint8_t hw_version; /* 硬件版本 */
  uint8_t reserved;   /* 保留 */
  uint8_t raw[MOTOR_FRAME_SIZE]; /* 原始 10 字节，失败时也用来看现场 */
} MotorVersion;

/* 模式切换/查询反馈（0xA0->0xA1、0x75->0x76） */
typedef struct
{
  uint8_t id;                     /* 反馈帧的 ID（= 查询时用的 ID） */
  uint8_t mode;                   /* 反馈帧 DATA[2]：切换后的实际模式，0x01 = 电流环 */
  uint8_t raw[MOTOR_FRAME_SIZE];  /* 原始 10 字节，失败时用来看现场 */
} MotorMode;

/* 其他反馈（0x74->0x75）：里程 / 位置 / 故障码 */
typedef struct
{
  uint8_t  id;                    /* 反馈帧的 ID（= 查询时用的 ID） */
  int32_t  mileage;               /* 里程圈数，-2147483647~2147483647，重新上电清 0 */
  uint16_t position;              /* 位置值 0~32767 对应 0~360° */
  uint8_t  fault;                 /* 故障码，位定义见 MOTOR_FAULT_xxx */
  uint8_t  raw[MOTOR_FRAME_SIZE]; /* 原始 10 字节，失败时用来看现场 */
} MotorStatus;

/* 绑定串口（传 &huart10）并清掉串口里的残留字节 */
void Motor_Init(UART_HandleTypeDef *huart);

/*
 * 发送一条 10 字节命令并校验回复（一问一答，内部阻塞）。
 *   motor_id  - 目标电机 ID（帧首字节）
 *   cmd       - 发送帧 DATA[1] 功能码
 *   payload   - DATA[2..8] 共 7 字节，传 NULL 表示全 0
 *   reply_cmd - 期望的反馈帧 DATA[1]
 *   raw       - 输出（可传 NULL）：收到的原始 10 字节；收不满时是已收到的部分
 *   reply     - 输出（可传 NULL）：反馈帧 DATA[2..8] 共 7 字节
 * 返回 1 = 成功（ID / 功能码 / CRC8 全部对得上），0 = 失败。
 */
uint8_t Motor_Transaction(uint8_t motor_id, uint8_t cmd, const uint8_t *payload,
                          uint8_t reply_cmd, uint8_t *raw, uint8_t *reply);

/*
 * 模式切换：发 0xA0，等 0xA1。
 *   mode   - MOTOR_MODE_xxx（使能用 MOTOR_MODE_ENABLE）
 *   result - 输出（可传 NULL）：反馈模式值和原始字节
 * 返回 1 = 收到合法反馈，0 = 失败（超时/校验不过）。
 */
uint8_t Motor_SetMode(uint8_t motor_id, uint8_t mode, MotorMode *result);

/* 使能电机（= SetMode(MOTOR_MODE_ENABLE)）。
   判成功看返回值：收到合法 0xA1 回帧即成功；result->mode 是使能后的实际模式
   （默认电流环 MOTOR_MODE_CURRENT），不是 0x08。 */
uint8_t Motor_Enable(uint8_t motor_id, MotorMode *result);

/* 失能电机（= SetMode(MOTOR_MODE_DISABLE)） */
uint8_t Motor_Disable(uint8_t motor_id, MotorMode *result);

/* 查询当前模式：发 0x75，等 0x76 */
uint8_t Motor_QueryMode(uint8_t motor_id, MotorMode *result);

/* 查询里程/位置/故障码：发 0x74，等 0x75 */
uint8_t Motor_QueryStatus(uint8_t motor_id, MotorStatus *status);

/*
 * 查询一台电机的版本号：发 0xFD -> 收 10 字节 0xFE 反馈 -> 校验 ID/功能码/CRC8。
 *   version  - 输出：收到的原始字节和解析结果
 * 返回 1 = 成功（version 已填好），0 = 失败（version->raw 里是收到的原始字节，
 * 没收到就是全 0，可以直接打出来定位问题）。整段是阻塞的，最长约 2*超时。
 */
uint8_t Motor_QueryVersion(uint8_t motor_id, MotorVersion *version);

#endif /* __MOTOR_H__ */
