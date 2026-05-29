/**
 * @file  app_usb_msc.c
 * @brief TinyUSB MSC Device — 将 eMMC 映射为 USB U 盘。
 *
 * 使用前提：
 *   1. eMMC 已通过 dev_emmcInitialize(0) 初始化完成。
 *   2. 本模块与 FatFS 同时挂载同一个 eMMC 卷时存在文件系统竞争风险，
 *      建议二者互斥使用（USB 连接时卸载 FatFS，断开后重新挂载）。
 *
 * USB 硬件：
 *   USB OTG FS（Synopsys DWC2），PA11(D-) / PA12(D+)，GPIO_AF10_OTG_FS
 *   中断：OTG_FS_IRQn → OTG_FS_IRQHandler → tud_int_handler(0)
 */

#include "app_usb_msc.h"

#include "tusb.h"
#include "class/msc/msc_device.h"

#include "main.h"           /* stm32f4xx_hal.h + project-wide defines */
#include "cmsis_os.h"
#include "elog.h"

#include "drv_emmc.h"
#include "dev_emmc.h"
#include "diskio.h"         /* DRESULT, RES_OK, BYTE, DWORD, UINT */
#include "app_log_manage.h"

/* ============================================================
 *  FreeRTOS task — 驱动 TinyUSB 事件循环
 * ============================================================ */
static uint8_t      task_usb_buf[1024];
static StaticTask_t task_usb_cb;
static const osThreadAttr_t s_task_usb_attr = {
    .name       = "task_usbMsc",
    .stack_mem  = task_usb_buf,
    .stack_size = sizeof(task_usb_buf),
    .cb_mem     = &task_usb_cb,
    .cb_size    = sizeof(task_usb_cb),
    .priority   = (osPriority_t)osPriorityHigh,
};

static void task_usbMsc(void *arg)
{
    (void)arg;
    for (;;)
    {
        tud_task();     /* OPT_OS_FREERTOS 下无事件时自动阻塞，不空转 */
    }
}

/* ============================================================
 *  Hardware + TinyUSB 初始化
 * ============================================================ */
void app_usbMscInit(void)
{
    /* 1. 使能时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
    tusb_rhport_init_t msc_dev_init = { .role  = TUSB_ROLE_DEVICE, .speed = TUSB_SPEED_FULL };

    /* 2. 配置 PA11(D-) / PA12(D+) 为 USB OTG FS 复用功能 */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_HIGH;
    gpio.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* 3. 设置 NVIC 优先级
     *    必须 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY（FreeRTOSConfig.h 中定义为 5）
     *    NVIC_EnableIRQ 由 TinyUSB dcd_init 内部调用，无需在此显式开启。 */
    HAL_NVIC_SetPriority(OTG_FS_IRQn, 0xAU, 0U);

    /* 4. 初始化 TinyUSB device stack（OTG FS = rhport 0） */
    tusb_init(BOARD_TUD_RHPORT, &msc_dev_init);

    /* 5. 创建事件处理任务 */
    osThreadNew(task_usbMsc, NULL, &s_task_usb_attr);

    log_i("TinyUSB MSC init OK — eMMC exposed as USB storage.");
}

/* ============================================================
 *  TinyUSB MSC 回调实现
 *  所有回调均在 task_usbMsc 任务上下文中执行，可以调用阻塞 API。
 * ============================================================ */

/** SCSI INQUIRY：向主机报告设备标识信息 */
void tud_msc_inquiry_cb(uint8_t lun,
                        uint8_t vendor_id[8],
                        uint8_t product_id[16],
                        uint8_t product_rev[4])
{
    (void)lun;
    memcpy(vendor_id,   "HZD     ", 8);
    memcpy(product_id,  "eMMC Storage    ", 16);
    memcpy(product_rev, "1.0 ", 4);
}

/** SCSI TEST UNIT READY：存储介质是否就绪 */
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
    (void)lun;
    return (drv_emmcGetState() == EMMC_TRANSFER_OK);
}

/** SCSI READ CAPACITY：总扇区数 + 扇区字节数 */
void tud_msc_capacity_cb(uint8_t   lun,
                          uint32_t *block_count,
                          uint16_t *block_size)
{
    (void)lun;
    HAL_MMC_CardInfoTypeDef card_info;
    HAL_MMC_CardCIDTypeDef  card_cid;
    drv_emmcGetInfo(&card_info, &card_cid);
    *block_count = card_info.LogBlockNbr;
    *block_size  = 512U;
}

/** 写保护状态（返回 false = 可写） */
bool tud_msc_is_writable_cb(uint8_t lun)
{
    (void)lun;
    return true;
}

/**
 * SCSI READ10：主机读扇区
 * @return 实际读取字节数（== bufsize 表示成功），-1 表示出错
 */
int32_t tud_msc_read10_cb(uint8_t  lun,
                           uint32_t lba,
                           uint32_t offset,
                           void    *buffer,
                           uint32_t bufsize)
{
    (void)lun;
    (void)offset;   /* 块设备访问 offset 始终为 0 */

    UINT block_count = (UINT)(bufsize / 512U);
    DRESULT res = dev_emmcRead(0U, (BYTE *)buffer, (DWORD)lba, block_count);
    return (res == RES_OK) ? (int32_t)bufsize : -1;
}

/**
 * SCSI WRITE10：主机写扇区
 * @return 实际写入字节数（== bufsize 表示成功），-1 表示出错
 */
int32_t tud_msc_write10_cb(uint8_t  lun,
                            uint32_t lba,
                            uint32_t offset,
                            uint8_t *buffer,
                            uint32_t bufsize)
{
    (void)lun;
    (void)offset;

    UINT block_count = (UINT)(bufsize / 512U);
    DRESULT res = dev_emmcWrite(0U, (const BYTE *)buffer, (DWORD)lba, block_count);
    return (res == RES_OK) ? (int32_t)bufsize : -1;
}

/* ============================================================
 *  TinyUSB 设备状态回调 — USB 与 FatFS 互斥
 * ============================================================ */

/** USB 被主机挂载：暂停日志写入，卸载 FatFS */
void tud_mount_cb(void)
{
    log_mgr_usb_mounted();
    log_i("USB MSC mounted by host");
}

/** USB 被主机卸载：重新挂载 FatFS，恢复日志写入 */
void tud_umount_cb(void)
{
    log_mgr_usb_unmounted();
    log_i("USB MSC unmounted by host");
}

/**
 * 自定义 SCSI 命令：不支持，返回 ILLEGAL REQUEST
 */
int32_t tud_msc_scsi_cb(uint8_t        lun,
                         uint8_t const  scsi_cmd[16],
                         void          *buffer,
                         uint16_t       bufsize)
{
    (void)lun; (void)scsi_cmd; (void)buffer; (void)bufsize;
    tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
    return -1;
}
