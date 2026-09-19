/**
  ******************************************************************************
  * @file    ota_service.c
  * @brief   App 侧 OTA 服务（RTOS 部分）
  *
  * 分工：
  *   ota_com.c     串口（中断收 + 环形缓冲）
  *   ota_link.c    COBS 组帧/解帧
  *   ota_host.c    协议命令（INFO/BEGIN/DATA/END/…，和 Bootloader 共用）
  *   ota_session.c 下载会话 + Flash 写入
  *   **本文件**     RTOS 任务 + 钩子（电机/日志）+ 启动确认 + 控制命令
  ******************************************************************************
  */

#include "ota_service.h"
#include "ota_com.h"
#include "ota_link.h"
#include "ota_host.h"
#include "ota_session.h"
#include "ota_meta.h"
#include "ota_flash.h"
#include "ota_crc.h"

#include "uart_log.h"
#include "ota_trace.h"
#include "motor_ctrl.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include <stdio.h>
#include <string.h>

/* 任务栈（字节，CMSIS-RTOS2 的 stack_size 单位是字节）。
   里面有 snprintf（一百多字节）+ HAL 调用，留 2 KB */
#define OTA_SVC_TASK_STACK      (512U * 4U)

/* Private variables ---------------------------------------------------------*/

static osThreadId_t s_task;
static uint8_t      s_muted;

static const osThreadAttr_t s_task_attr =
{
    .name       = "otaSvc",
    .stack_size = OTA_SVC_TASK_STACK,
    .priority   = (osPriority_t)osPriorityAboveNormal,
};

/* Private functions ---------------------------------------------------------*/

/* 中断里被叫醒（在 USART1 的 ISR 上下文里，优先级 5 → 可以调 FromISR API） */
static void OtaService_RxHook(void)
{
    BaseType_t woken = pdFALSE;

    /*
     * ⚠⚠ 调度器还没跑起来之前，**绝对不能**碰 FreeRTOS 的 API！
     *
     * 这个钩子是在 USART1 中断里跑的，而 OtaCom_Init()（在 MX_FREERTOS_Init 里）
     * 一早就把接收中断武装好了 —— 从那一刻到 osKernelStart() 之间是个危险窗口：
     * 只要主机在那段时间发一个字节（比如上位机在反复重试 info），
     * 下面这两句就会：vTaskNotifyGiveFromISR 置位 + portYIELD_FROM_ISR 触发 PendSV
     * → PendSV 优先级比 USART1 还高，立刻切进 pxCurrentTCB
     * → 那会儿就绪表/延时表还没初始化 → HardFault → 整块板子变哑巴。
     * 实测症状：App 打完 [ota_com] USART1 up 就没了（就是被 info 的帧打死的）。
     *
     * 早到的字节不会丢：它在环形缓冲里等着，任务一起来就会处理。
     */
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING)
    {
        return;
    }

    if (s_task != NULL)
    {
        (void)vTaskNotifyGiveFromISR((TaskHandle_t)s_task, &woken);
    }

    portYIELD_FROM_ISR(woken);
}

/* ---- 给 ota_host 的钩子 --------------------------------------------------- */

static uint8_t OtaService_GetRunningSlot(void)
{
    return OTA_APP_SLOT;    /* 编译期就知道自己链在哪个槽 */
}

static int8_t OtaService_CheckTargetSlot(uint8_t slot)
{
    /* 正在跑的那个槽绝对不能写：擦掉自己 = 立刻变砖。
       也正是这条规则保证了 A/B 双槽"永远有一个是好的"。 */
    if (slot == OTA_APP_SLOT)
    {
        return OTA_E_NOTALLOWED;
    }

    return OTA_OK;
}

static void OtaService_Log(const char *line)
{
    UartLog_Print(line);
}

static void OtaService_OnSessionStart(void)
{
    char line[96];

    (void)snprintf(line, sizeof(line),
                   "ota: start, motor disabled + poll paused + log muted\r\n");
    UartLog_Print(line);

    MotorCtrl_SetOtaMode(1U);    /* 失能电机 + 停轮询 */
    OtaService_MuteLog(1U);      /* 别让日志抢带宽 */
}

static void OtaService_OnSessionStop(void)
{
    MotorCtrl_SetOtaMode(0U);
    OtaService_MuteLog(0U);
}

/* 发帧时和文本日志串行化：不然一行日志插进帧中间 → 上行 CRC 不过、白重传一次 */
static void OtaService_Lock(void)
{
    UartLog_Lock();
}

static void OtaService_Unlock(void)
{
    UartLog_Unlock();
}

static const ota_host_hooks_t s_hooks =
{
    .get_running_slot   = OtaService_GetRunningSlot,
    .check_target_slot  = OtaService_CheckTargetSlot,
    .on_session_start   = OtaService_OnSessionStart,
    .on_session_stop    = OtaService_OnSessionStop,
    .log                = OtaService_Log,
    .lock               = OtaService_Lock,
    .unlock             = OtaService_Unlock,
};

/* ---- 启动确认 ------------------------------------------------------------- */

/*
 * 跑起来 2 s 之后做两件事：
 *   1) 把自己的镜像（长度 + CRC32）登记到元数据里 —— 首次用 ST-Link 烧进来的镜像
 *      就是这么"报到"的（Bootloader 之前只知道"向量表看着像 App"）。
 *   2) 把 boot_attempts 清零 = 告诉 Bootloader"这个新固件跑起来了"。
 *      没有这一步，Bootloader 数到 3 次就会判它坏、回滚。
 */
static void OtaService_ConfirmBoot(void)
{
    uint8_t    slot = OTA_APP_SLOT;
    uint32_t   size = OTA_APP_IMAGE_SIZE;
    uint32_t   crc  = 0U;
    ota_meta_t m;
    char       line[160];
    int8_t     st;

    if (slot >= OTA_SLOT_COUNT)
    {
        return;    /* 链接基址不在任何槽里：要么 link 步骤出错，要么 .ld 没配对 */
    }

    if (OtaFlash_SlotCrc(slot, size, &crc) != OTA_OK)
    {
        return;
    }

    if (OtaMeta_Load(&m) < 0)
    {
        return;
    }

    if ((m.active_slot == slot) && (m.boot_slot == slot) && (m.boot_attempts == 0U) &&
        (m.slot_size[slot] == size) && (m.slot_crc[slot] == crc))
    {
        return;    /* 都登记过了：不写 Flash（省寿命） */
    }

    m.active_slot   = slot;
    m.boot_slot     = slot;
    m.boot_attempts = 0U;
    m.flags         = (uint8_t)(m.flags & (uint8_t)~0x01U);   /* 清"停在 BL"标志 */
    m.slot_size[slot] = size;
    m.slot_crc[slot]  = crc;

    if (m.slot_ver[slot] == 0U)
    {
        m.slot_ver[slot] = OTA_FW_VERSION;
    }
    m.fw_version = m.slot_ver[slot];

    st = OtaMeta_Write(&m);

    if (st == OTA_OK)
    {
        (void)snprintf(line, sizeof(line),
                       "ota: boot confirmed: slot=%c, size=%lu, crc=0x%08lX, ver=0x%08lX\r\n",
                       (slot == OTA_SLOT_A) ? 'A' : 'B',
                       (unsigned long)size, (unsigned long)crc,
                       (unsigned long)m.slot_ver[slot]);
        UartLog_Print(line);
    }
    else
    {
        /* ⚠ 千万别静默：这条写不进去，BL 下次就只能靠"向量表看着像 App"兜底，
           而且 A/B 切换 / 回滚 / 断点续传全部靠元数据 —— 失败必须看得见 */
        (void)snprintf(line, sizeof(line),
                       "ota: boot confirm FAILED (st=%d)：元数据没写进去\r\n",
                       (int)st);
        UartLog_Print(line);
    }
}

/* ---- 控制命令（不升级时的"控制信息"通道） --------------------------------- */

static void OtaService_Ctrl(uint8_t seq, const uint8_t *pl, uint16_t len)
{
    uint8_t  rsp[5];
    uint32_t out = 0U;

    if (len < 5U)
    {
        rsp[0] = (uint8_t)OTA_E_PARAM;
    }
    else
    {
        rsp[0] = MotorCtrl_RemoteCmd(pl[0], Ota_GetLe32(&pl[1]), &out);
    }

    Ota_PutLe32(&rsp[1], out);

    UartLog_Lock();
    OtaLink_Send((uint8_t)(OTA_T_CTRL | 0x80U), seq, rsp, 5U);
    UartLog_Unlock();
}

static void OtaService_Status(uint8_t seq)
{
    int32_t  mileage  = 0;
    uint16_t position = 0U;
    uint8_t  fault    = 0U;
    uint8_t  mode     = 0U;
    uint8_t  rsp[9];
    uint8_t  status;

    status = MotorCtrl_RemoteStatus(&mileage, &position, &fault, &mode);

    rsp[0] = status;
    Ota_PutLe32(&rsp[1], (uint32_t)mileage);
    Ota_PutLe16(&rsp[5], position);
    rsp[7] = fault;
    rsp[8] = mode;

    UartLog_Lock();
    OtaLink_Send((uint8_t)(OTA_T_STATUS | 0x80U), seq, rsp, 9U);
    UartLog_Unlock();
}

/* ---- 帧分发 --------------------------------------------------------------- */

static void OtaService_OnFrame(uint8_t type, uint8_t seq, const uint8_t *pl, uint16_t len)
{
    uint8_t reboot = 0U;

    /* INFO/BEGIN/DATA/END/ABORT/ERASE/ROLLBACK/REBOOT 都在 ota_host.c（和 BL 共用） */
    if (OtaHost_HandleFrame(type, seq, pl, len, &reboot) != 0U)
    {
        if (reboot != 0U)
        {
            /* 回复已经同步发完了（HAL_UART_Transmit 是阻塞的），
               再等一下让无线模块把最后几个字节送出去 */
            osDelay(300U);
            NVIC_SystemReset();
        }

        return;
    }

    switch (type)
    {
        case OTA_T_CTRL:
            OtaService_Ctrl(seq, pl, len);
            break;

        case OTA_T_STATUS:
            OtaService_Status(seq);
            break;

        default:
        {
            uint8_t rsp = (uint8_t)OTA_E_PARAM;

            UartLog_Lock();
            OtaLink_Send((uint8_t)(type | 0x80U), seq, &rsp, 1U);
            UartLog_Unlock();
            break;
        }
    }
}

/* ---- 任务 ----------------------------------------------------------------- */

static void OtaService_Task(void *argument)
{
    uint8_t  buf[128];
    uint32_t start   = HAL_GetTick();
    uint8_t  confirmed = 0U;

    (void)argument;

    /*
     * ⚠ 接收中断**必须等到这里**才能开（不能放在 OtaService_Init 里）：
     *   中断里会调 vTaskNotifyGiveFromISR + portYIELD_FROM_ISR，而调度器还没跑起来时
     *   这一下 PendSV 会直接切进 pxCurrentTCB，那时就绪/延时表还是空的 → HardFault。
     *   进到任务函数的这一刻，调度器已经肯定在跑了。
     *   （主机在启动期间发的帧会丢，但工具会重试，无所谓。）
     */
    OtaCom_StartRx();

    OtaTrace_Text("[app] K: otaSvc task running (USART1 rx armed)\r\n");

    for (;;)
    {
        /* 等中断叫醒；100 ms 只是兜底（顺便当心跳给看门狗用） */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));

        uint16_t n;

        while ((n = OtaCom_Read(buf, (uint16_t)sizeof(buf))) > 0U)
        {
            for (uint16_t i = 0U; i < n; i++)
            {
                OtaLink_Feed(buf[i]);
            }
        }

        OtaHost_Tick();

        if ((confirmed == 0U) && ((HAL_GetTick() - start) >= OTA_CONFIRM_DELAY_MS))
        {
            confirmed = 1U;
            OtaService_ConfirmBoot();
        }
    }
}

/* Exported functions --------------------------------------------------------*/

void OtaService_MuteLog(uint8_t mute)
{
    s_muted = (mute != 0U) ? 1U : 0U;

    UartLog_SetEnabled((s_muted != 0U) ? 0U : 1U);
}

uint8_t OtaService_Active(void)
{
    return OtaHost_SessionActive();
}

void OtaService_Init(void)
{
    /* 串口（重复调用是安全的：OtaCom_Init 内部有 s_inited 判断） */
    OtaCom_Init(OTA_PORT_BAUD);

    OtaLink_Init(OtaService_OnFrame);
    OtaHost_Init(&s_hooks);
    OtaCom_SetRxHook(OtaService_RxHook);

    s_task = osThreadNew(OtaService_Task, NULL, &s_task_attr);

    if (s_task == NULL)
    {
        /* 一般是 configTOTAL_HEAP_SIZE 不够了：日志口这时已经能用，报出来 */
        UartLog_Print("ota: ERROR: otaSvc task not created (FreeRTOS heap too small?)\r\n");
    }
}
