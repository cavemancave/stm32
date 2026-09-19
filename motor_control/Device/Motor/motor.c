/**
  ******************************************************************************
  * @file    motor.c
  * @brief   串口电机驱动：一问一答，固定 10 字节帧 + CRC-8/MAXIM
  ******************************************************************************
  * 时序/流程（见 Device/Motor/inc/motor.h 里的协议说明）:
  *   1. 清掉串口里上一次留下的字节和错误标志
  *   2. 发 10 字节: ID | 功能码 | DATA[2..8] | CRC8
  *   3. 收 10 字节: ID | 反馈码 | DATA[2..8] | CRC8
  *   4. 校验 ID、反馈码、CRC-8/MAXIM
  * 用的是阻塞收发，一问一答的节奏下最简单可靠；10 字节 @38400 只要 ~2.6ms。
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor.h"

#include <string.h>

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *motor_uart = NULL;

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  CRC8: CRC-8/MAXIM（手册"CRC8 值"一节写明）
  *         多项式 x^8+x^5+x^4+1 = 0x31，输入/输出都反射，初值 0x00，不异或。
  *         反射算法等价于右移、用位反转后的多项式 0x8C（= MOTOR_CRC8_POLY）。
  * @param  data/length - 参与校验的数据（DATA[0..8]，不含 CRC 自身）
  * @retval CRC8 值
  */
static uint8_t Motor_Crc8(const uint8_t *data, uint16_t length)
{
    uint8_t crc = 0x00U;

    for (uint16_t i = 0U; i < length; i++)
    {
        crc ^= data[i];

        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = ((crc & 0x01U) != 0U) ? (uint8_t)((crc >> 1) ^ MOTOR_CRC8_POLY)
                                        : (uint8_t)(crc >> 1);
        }
    }

    return crc;
}

/**
  * @brief  清空串口接收，让每次一问一答都从干净状态开始
  * @note   上一轮超时/校验失败时可能还有半帧残留在 RX 里，不清掉下一轮就会错位；
  *         接收过程中出现的溢出/帧错误标志也一起清掉。
  * @retval None
  */
static void Motor_FlushRx(void)
{
    uint8_t dummy;

    /* 一字节一字节地读，读到读不出来了为止（残留不会太多） */
    while (HAL_UART_Receive(motor_uart, &dummy, 1U, 1U) == HAL_OK)
    {
        /* 丢掉 */
    }

    motor_uart->ErrorCode = HAL_UART_ERROR_NONE;
    __HAL_UART_CLEAR_FLAG(motor_uart,
                          UART_CLEAR_OREF | UART_CLEAR_FEF | UART_CLEAR_NEF | UART_CLEAR_PEF);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  绑定电机所在串口并清空接收
  * @param  huart - 电机串口句柄，传 &huart10
  * @retval None
  */
void Motor_Init(UART_HandleTypeDef *huart)
{
    motor_uart = huart;

    if (motor_uart != NULL)
    {
        Motor_FlushRx();
    }
}

/**
  * @brief  一问一答：发一条 10 字节命令，收一条 10 字节回复并校验
  * @param  motor_id  - 目标电机 ID（帧首字节，反馈里也必须一致）
  * @param  cmd       - 发送帧 DATA[1] 功能码
  * @param  payload   - DATA[2..8] 共 7 字节，传 NULL 表示全 0
  * @param  reply_cmd - 期望的反馈帧 DATA[1]
  * @param  raw       - 输出（可 NULL）：收到的原始 10 字节，失败时看现场用
  * @param  reply     - 输出（可 NULL）：反馈帧 DATA[2..8] 共 7 字节
  * @retval 1 = 成功，0 = 失败
  */
uint8_t Motor_Transaction(uint8_t motor_id, uint8_t cmd, const uint8_t *payload,
                          uint8_t reply_cmd, uint8_t *raw, uint8_t *reply)
{
    uint8_t frame[MOTOR_FRAME_SIZE];
    uint8_t rx[MOTOR_FRAME_SIZE] = {0};
    const uint8_t payload_zero[MOTOR_FRAME_PAYLOAD_SIZE] = {0};
    HAL_StatusTypeDef rx_status;

    if (motor_uart == NULL)
    {
        return 0U;
    }

    if (payload == NULL)
    {
        payload = payload_zero;
    }

    /* 发送: ID | 功能码 | DATA[2..8] | CRC8 */
    frame[0] = motor_id;
    frame[1] = cmd;

    for (uint8_t i = 0U; i < MOTOR_FRAME_PAYLOAD_SIZE; i++)
    {
        frame[2U + i] = payload[i];
    }

    frame[MOTOR_FRAME_SIZE - 1U] = Motor_Crc8(frame, MOTOR_FRAME_SIZE - 1U);

    /* raw 清零：失败时全 0 = 一个字节都没收到 */
    if (raw != NULL)
    {
        for (uint8_t i = 0U; i < MOTOR_FRAME_SIZE; i++)
        {
            raw[i] = 0x00U;
        }
    }

    Motor_FlushRx();

    if (HAL_UART_Transmit(motor_uart, frame, MOTOR_FRAME_SIZE, MOTOR_FRAME_TIMEOUT_MS) != HAL_OK)
    {
        return 0U;
    }

    /* 收 10 字节反馈；收不满也把已收到的部分留进 raw，方便定位接线/波特率问题 */
    rx_status = HAL_UART_Receive(motor_uart, rx, MOTOR_FRAME_SIZE, MOTOR_FRAME_TIMEOUT_MS);

    if (raw != NULL)
    {
        memcpy(raw, rx, MOTOR_FRAME_SIZE);
    }

    if (rx_status != HAL_OK)
    {
        Motor_FlushRx();
        return 0U;
    }

    /* 帧头校验：反馈的 ID 必须就是刚问的那台，否则可能是别的电机在抢答/串口错位 */
    if ((rx[0] != motor_id) || (rx[1] != reply_cmd))
    {
        return 0U;
    }

    /* CRC-8/MAXIM 校验 */
    if (Motor_Crc8(rx, MOTOR_FRAME_SIZE - 1U) != rx[MOTOR_FRAME_SIZE - 1U])
    {
        return 0U;
    }

    if (reply != NULL)
    {
        for (uint8_t i = 0U; i < MOTOR_FRAME_PAYLOAD_SIZE; i++)
        {
            reply[i] = rx[2U + i];
        }
    }

    return 1U;
}

/**
  * @brief  模式切换 / 使能 / 失能：发 0xA0，等 0xA1
  * @param  motor_id - 目标电机 ID
  * @param  mode     - MOTOR_MODE_xxx
  * @param  result   - 输出（可 NULL）：反馈的模式值和原始字节
  * @retval 1 = 收到合法反馈，0 = 失败
  */
uint8_t Motor_SetMode(uint8_t motor_id, uint8_t mode, MotorMode *result)
{
    uint8_t payload[MOTOR_FRAME_PAYLOAD_SIZE] = {0};
    uint8_t reply[MOTOR_FRAME_PAYLOAD_SIZE];
    uint8_t *raw = NULL;
    uint8_t  ok;

    if (result != NULL)
    {
        raw          = result->raw;
        result->id   = motor_id;
        result->mode = 0x00U;
    }

    payload[0] = mode;

    ok = Motor_Transaction(motor_id, MOTOR_CMD_SET_MODE, payload,
                           MOTOR_REPLY_SET_MODE, raw, reply);

    if ((ok != 0U) && (result != NULL))
    {
        result->mode = reply[0];
    }

    return ok;
}

/**
  * @brief  使能电机（使能后默认电流环）
  * @note   电机刚上电时要过一段时间才会应答，需要重试的话由调用方循环。
  *         返回 1 = 收到合法的 0xA1 回帧（即使能指令被接受）；
  *         result->mode 是使能之后的实际模式（默认 0x01 电流环），
  *         电机不会把 0x08 回显回来，所以不要用 mode == 0x08 判成功。
  * @retval 1 = 收到合法反馈，0 = 失败
  */
uint8_t Motor_Enable(uint8_t motor_id, MotorMode *result)
{
    return Motor_SetMode(motor_id, MOTOR_MODE_ENABLE, result);
}

/**
  * @brief  电机失能
  * @retval 1 = 收到合法反馈，0 = 失败
  */
uint8_t Motor_Disable(uint8_t motor_id, MotorMode *result)
{
    return Motor_SetMode(motor_id, MOTOR_MODE_DISABLE, result);
}

/**
  * @brief  查询当前模式：发 0x75，等 0x76
  * @param  motor_id - 目标电机 ID
  * @param  result   - 输出：模式值和原始字节
  * @retval 1 = 成功，0 = 失败
  */
uint8_t Motor_QueryMode(uint8_t motor_id, MotorMode *result)
{
    uint8_t reply[MOTOR_FRAME_PAYLOAD_SIZE];
    uint8_t *raw = NULL;
    uint8_t  ok;

    if (result == NULL)
    {
        return 0U;
    }

    raw          = result->raw;
    result->id   = motor_id;
    result->mode = 0x00U;

    ok = Motor_Transaction(motor_id, MOTOR_CMD_GET_MODE, NULL,
                           MOTOR_REPLY_GET_MODE, raw, reply);

    if (ok != 0U)
    {
        result->mode = reply[0];
    }

    return ok;
}

/**
  * @brief  查询里程 / 位置 / 故障码：发 0x74，等 0x75
  * @param  motor_id - 目标电机 ID
  * @param  status   - 输出：解析结果和原始字节
  * @retval 1 = 成功，0 = 失败
  */
uint8_t Motor_QueryStatus(uint8_t motor_id, MotorStatus *status)
{
    uint8_t reply[MOTOR_FRAME_PAYLOAD_SIZE];
    uint8_t *raw;
    uint32_t mileage;

    if (status == NULL)
    {
        return 0U;
    }

    raw = status->raw;

    status->id       = motor_id;
    status->mileage  = 0;
    status->position = 0U;
    status->fault    = 0x00U;

    if (Motor_Transaction(motor_id, MOTOR_CMD_GET_STATUS, NULL,
                          MOTOR_REPLY_GET_STATUS, raw, reply) == 0U)
    {
        return 0U;
    }

    /* DATA[2..5] = 里程圈数（有符号 32 位，大端），DATA[6..7] = 位置，DATA[8] = 故障码 */
    mileage = ((uint32_t)reply[0] << 24) | ((uint32_t)reply[1] << 16) |
              ((uint32_t)reply[2] << 8)  |  (uint32_t)reply[3];

    status->mileage  = (int32_t)mileage;
    status->position = (uint16_t)(((uint16_t)reply[4] << 8) | (uint16_t)reply[5]);
    status->fault    = reply[6];

    return 1U;
}

/**
  * @brief  查询一台电机的版本号：发 0xFD，等 0xFE
  * @param  motor_id - 目标电机 ID
  * @param  version  - 输出，收到的原始字节和解析结果
  * @retval 1 = 成功，0 = 失败
  */
uint8_t Motor_QueryVersion(uint8_t motor_id, MotorVersion *version)
{
    uint8_t reply[MOTOR_FRAME_PAYLOAD_SIZE];

    if (version == NULL)
    {
        return 0U;
    }

    version->id         = motor_id;
    version->year       = 0x00U;
    version->month      = 0x00U;
    version->day        = 0x00U;
    version->model      = 0x00U;
    version->fw_version = 0x00U;
    version->hw_version = 0x00U;
    version->reserved   = 0x00U;

    if (Motor_Transaction(motor_id, MOTOR_CMD_VERSION_QUERY, NULL,
                          MOTOR_REPLY_VERSION, version->raw, reply) == 0U)
    {
        return 0U;
    }

    /* 解析: 年 | 月 | 日 | 型号 | 软件版本 | 硬件版本 | 保留 */
    version->year       = reply[0];
    version->month      = reply[1];
    version->day        = reply[2];
    version->model      = reply[3];
    version->fw_version = reply[4];
    version->hw_version = reply[5];
    version->reserved   = reply[6];

    return 1U;
}
