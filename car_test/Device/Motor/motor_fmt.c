/**
  ******************************************************************************
  * @file    motor_fmt.c
  * @brief   电机协议值的人话化：模式名 / 故障码 / 角度换算（纯函数，无 RTOS）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "motor_fmt.h"
#include "motor.h"

#include <stdio.h>
#include <string.h>

/* Private defines ---------------------------------------------------------*/

/* 位置值满量程：32767 -> 360° */
#define MOTOR_POSITION_FULL_SCALE   32767U

/* Exported functions --------------------------------------------------------*/

const char *Motor_ModeName(uint8_t mode)
{
    switch (mode)
    {
        case MOTOR_MODE_OPEN_LOOP:      return "open loop";
        case MOTOR_MODE_CURRENT:        return "current loop";
        case MOTOR_MODE_SPEED:          return "speed loop";
        case MOTOR_MODE_POSITION:       return "position loop";
        case MOTOR_MODE_ENABLE:         return "enable";
        case MOTOR_MODE_DISABLE:        return "disable";
        case MOTOR_MODE_TURN_BACK_150:  return "turn back 150deg";
        case MOTOR_MODE_LINK_LOSS_ON:   return "link-loss on";
        case MOTOR_MODE_LINK_LOSS_OFF:  return "link-loss off";
        default:                        return "unknown";
    }
}

void Motor_FaultText(uint8_t fault, char *out, size_t out_size)
{
    static const struct
    {
        uint8_t     bit;
        const char *name;
    } faults[] =
    {
        { MOTOR_FAULT_HALL,        "hall"        },
        { MOTOR_FAULT_OVERCURRENT, "overcurrent" },
        { MOTOR_FAULT_STALL,       "stall"       },
        { MOTOR_FAULT_OVERTEMP,    "overtemp"    },
        { MOTOR_FAULT_LINK_LOSS,   "link-loss"   },
        { MOTOR_FAULT_VOLTAGE,     "voltage"     },
    };

    uint8_t known = 0x00U;
    size_t  used  = 0U;

    if ((out == NULL) || (out_size == 0U))
    {
        return;
    }

    out[0] = '\0';

    for (uint8_t i = 0U; i < (sizeof(faults) / sizeof(faults[0])); i++)
    {
        if ((fault & faults[i].bit) != 0U)
        {
            (void)snprintf(&out[used], out_size - used, "%s%s",
                           (used != 0U) ? "|" : "", faults[i].name);
            used  = strlen(out);
            known = (uint8_t)(known | faults[i].bit);
        }
    }

    /* 已知位之外还有置位（手册没定义的位）就标个 other */
    if ((uint8_t)(fault & (uint8_t)(~known)) != 0x00U)
    {
        (void)snprintf(&out[used], out_size - used, "%sother", (used != 0U) ? "|" : "");
        used = strlen(out);
    }

    if (used == 0U)
    {
        (void)snprintf(out, out_size, "none");
    }
}

uint16_t Motor_AngleToPosition(uint16_t angle_deg)
{
    if (angle_deg > 360U)
    {
        angle_deg %= 360U;
    }

    return (uint16_t)(((uint32_t)MOTOR_POSITION_FULL_SCALE * angle_deg) / 360U);
}

uint16_t Motor_PositionToDeciDeg(uint16_t position)
{
    if (position > MOTOR_POSITION_FULL_SCALE)
    {
        position = MOTOR_POSITION_FULL_SCALE;
    }

    /* 位置值 -> 0.1°，即 x * 3600 / 32767 */
    return (uint16_t)(((uint32_t)position * 3600U) / MOTOR_POSITION_FULL_SCALE);
}
