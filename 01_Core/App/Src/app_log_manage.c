/**
 * @file  app_log_manage.c
 * @brief 业务日志管理模块 — 将运行日志写入 eMMC (FatFS)
 *
 * 记录内容：基站开机、语音暂停/开启、CAN 通信状态、
 *          报警标签信息（ID/信号强度/来源）、系统参数等。
 *
 * 文件结构：0:/LOG/YYMM/DDA{id}.LOG
 * 日志行格式：[YYYY-MM-DD] [HH:MM:SS] [TYPE] content\r\n
 *
 * 通过 FreeRTOS 消息队列实现异步写入，不阻塞业务任务。
 * USB MSC 挂载期间自动暂停写入，卸载后恢复。
 */

#include "app_log_manage.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "ff.h"
#include "cmsis_os.h"
#include "elog.h"

#include "dev_emmc.h"
#include "dev_linkFatFs.h"
#include "dev_rx8130ce.h"
#include "dw_instance.h"        /* anc_id */

// #define LOG_TAG "LOG_MGR"

/* ============================================================
 *  配置
 * ============================================================ */
#define LOG_MGR_ROOT_DIR    "0:/LOG"
#define LOG_MGR_QUEUE_DEPTH 16          /* 消息队列深度 */
#define LOG_MGR_LINE_MAX    128         /* 单条日志 body 最大长度 */
#define LOG_MGR_FLUSH_MS    500         /* 定时 flush 间隔 */
#define LOG_MGR_MIN_FREE_MB 10          /* eMMC 最小保留空间 */

/* 类型标签字符串 */
static const char *type_str[] = {
    [LOG_MGR_SYS] = "SYS",
    [LOG_MGR_ALM] = "ALM",
    [LOG_MGR_COM] = "COM",
};

/* ============================================================
 *  日志条目结构（队列消息体）
 * ============================================================ */
typedef struct {
    uint16_t year;              /* 完整年份 2000+rx8130.year */
    uint8_t  month;
    uint8_t  day;
    uint8_t  hours;
    uint8_t  minutes;
    uint8_t  seconds;
    uint8_t  type;              /* log_mgr_type_t */
    char     body[LOG_MGR_LINE_MAX];
} log_entry_t;

/* ============================================================
 *  FatFS
 * ============================================================ */
static FATFS   logMgr_fatfs;
static FIL     logMgr_file;
static char    logMgr_drivePath[4];
static uint8_t logMgr_fileOpen;         /* 文件是否已打开 */
static uint8_t logMgr_curDay;           /* 当前打开文件对应的日期 */

/* USB 互斥标志 */
static volatile uint8_t logMgr_usbActive;  /* 1 = USB MSC 已挂载，暂停写入 */

/* ============================================================
 *  FreeRTOS 队列
 * ============================================================ */
static osMessageQueueId_t    queue_logMgr;
static log_entry_t           queue_logMgr_buf[LOG_MGR_QUEUE_DEPTH];
static StaticQueue_t         queue_logMgr_cb;
static const osMessageQueueAttr_t queue_logMgr_attr = {
    .name    = "queue_logMgr",
    .mq_mem  = queue_logMgr_buf,
    .mq_size = sizeof(queue_logMgr_buf),
    .cb_mem  = &queue_logMgr_cb,
    .cb_size = sizeof(queue_logMgr_cb),
};

/* ============================================================
 *  FreeRTOS 写入任务
 * ============================================================ */
static uint8_t      task_logMgr_buf[1024];
static StaticTask_t task_logMgr_cb;
static const osThreadAttr_t task_logMgr_attr = {
    .name       = "task_logMgr",
    .stack_mem  = task_logMgr_buf,
    .stack_size = sizeof(task_logMgr_buf),
    .cb_mem     = &task_logMgr_cb,
    .cb_size    = sizeof(task_logMgr_cb),
    .priority   = (osPriority_t)osPriorityBelowNormal,
};

/* ============================================================
 *  内部函数
 * ============================================================ */

/**
 * @brief 确保 0:/LOG/YYMM 目录存在
 */
static FRESULT log_mgr_ensure_dir(uint16_t year, uint8_t month)
{
    char path[16];
    FRESULT res;

    /* 创建根目录 0:/LOG */
    res = f_mkdir(LOG_MGR_ROOT_DIR);
    if (res != FR_OK && res != FR_EXIST)
    {
        return res;
    }

    /* 创建月份子目录 0:/LOG/YYMM（取年份后两位） */
    snprintf(path, sizeof(path), "%s/%02d%02d",
             LOG_MGR_ROOT_DIR, year % 100, month);
    res = f_mkdir(path);
    if (res != FR_OK && res != FR_EXIST)
    {
        return res;
    }

    return FR_OK;
}

/**
 * @brief 打开（或切换）当天的日志文件
 *        文件名：DDA{id}.LOG（8.3 格式），如 11A01.LOG
 */
static FRESULT log_mgr_open_file(uint16_t year, uint8_t month, uint8_t day)
{
    char path[32];
    FRESULT res;

    /* 如果已打开且日期未变，直接返回 */
    if (logMgr_fileOpen && logMgr_curDay == day)
    {
        return FR_OK;
    }

    /* 关闭旧文件 */
    if (logMgr_fileOpen)
    {
        f_sync(&logMgr_file);
        f_close(&logMgr_file);
        logMgr_fileOpen = 0;
    }

    /* 确保目录存在 */
    res = log_mgr_ensure_dir(year, month);
    if (res != FR_OK)
    {
        return res;
    }

    /* 生成文件路径：0:/LOG/YYMM/DDA{id}.LOG */
    snprintf(path, sizeof(path), "%s/%02d%02d/%02dA%02d.LOG",
             LOG_MGR_ROOT_DIR, year % 100, month, day, anc_id);

    /* 打开或创建（追加模式） */
    res = f_open(&logMgr_file, path, FA_OPEN_APPEND | FA_WRITE);
    if (res == FR_OK)
    {
        logMgr_fileOpen = 1;
        logMgr_curDay   = day;
    }

    return res;
}

/**
 * @brief 将一条日志写入文件
 */
static void log_mgr_write_entry(const log_entry_t *entry)
{
    char line[LOG_MGR_LINE_MAX + 48];
    UINT bw;
    int len;

    len = snprintf(line, sizeof(line),
                   "[%04d-%02d-%02d] [%02d:%02d:%02d] [%s] %s\r\n",
                   entry->year, entry->month, entry->day,
                   entry->hours, entry->minutes, entry->seconds,
                   type_str[entry->type],
                   entry->body);

    if (len > 0)
    {
        f_write(&logMgr_file, line, (UINT)len, &bw);
    }
}

/**
 * @brief 写入任务：从队列取出日志条目并写入 eMMC
 */
static void task_logMgrProcess(void *arg)
{
    (void)arg;
    log_entry_t entry;
    uint32_t last_flush = osKernelGetTickCount();

    for (;;)
    {
        /* 从队列取日志，超时时做 flush */
        osStatus_t st = osMessageQueueGet(queue_logMgr, &entry, NULL, LOG_MGR_FLUSH_MS);

        if (logMgr_usbActive)
        {
            continue;       /* USB 挂载期间丢弃 */
        }

        if (st == osOK)
        {
            FRESULT res = log_mgr_open_file(entry.year, entry.month, entry.day);
            if (res == FR_OK)
            {
                log_mgr_write_entry(&entry);
            }
        }

        /* 定时 flush */
        uint32_t tick = osKernelGetTickCount();
        if (logMgr_fileOpen && (tick - last_flush >= LOG_MGR_FLUSH_MS))
        {
            f_sync(&logMgr_file);
            last_flush = tick;
        }
    }
}

/* ============================================================
 *  公共 API
 * ============================================================ */

/**
 * @brief log写入emmc初始化，链接fatfs驱动，挂载路径，创建消息队列和写入任务
 */
int log_mgr_init(void)
{
    /* 链接 FatFS 驱动 */
    if (dev_FATFS_LinkDriver(&emmc_driver, logMgr_drivePath) != 0)
    {
        log_e("log_mgr: link driver failed");
        return -1;
    }

    /* 挂载 */
    FRESULT res = f_mount(&logMgr_fatfs, logMgr_drivePath, 1);
    if (res != FR_OK)
    {
        log_e("log_mgr: mount failed (%d)", res);
        return -1;
    }

    logMgr_fileOpen  = 0;
    logMgr_usbActive = 0;

    /* 创建消息队列 */
    queue_logMgr = osMessageQueueNew(LOG_MGR_QUEUE_DEPTH,
                                     sizeof(log_entry_t),
                                     &queue_logMgr_attr);
    if (queue_logMgr == NULL)
    {
        log_e("log_mgr: queue create failed");
        return -1;
    }

    /* 创建写入任务 */
    osThreadNew(task_logMgrProcess, NULL, &task_logMgr_attr);

    log_i("log_mgr: init OK, drive=%s", logMgr_drivePath);
    return 0;
}

void log_mgr_write(log_mgr_type_t type, uint8_t tag_id, const char *fmt, ...)
{
    if (queue_logMgr == NULL)
    {
        return;
    }

    log_entry_t entry;
    rx8130ce_time_t now;

    /* 采集完整时间戳（在调用者上下文） */
    dev_rx8130ceGetDateTime(&now);
    entry.year    = now.year + 2000;
    entry.month   = now.month;
    entry.day     = now.day;
    entry.hours   = now.hours;
    entry.minutes = now.minutes;
    entry.seconds = now.seconds;
    entry.type    = (uint8_t)type;

    /* 组装 body：先写标签标识，再写用户内容 */
    int offset = 0;
    if (tag_id != LOG_MGR_NO_TAG)
    {
        offset = snprintf(entry.body, sizeof(entry.body), "[T%02d] ", tag_id);
    }

    va_list ap;
    va_start(ap, fmt);
    vsnprintf(entry.body + offset, sizeof(entry.body) - offset, fmt, ap);
    va_end(ap);

    /* 入队，队满则丢弃（不阻塞业务） */
    osMessageQueuePut(queue_logMgr, &entry, 0, 0);
}

void log_mgr_usb_mounted(void)
{
    logMgr_usbActive = 1;

    /* 关闭文件并卸载 FatFS，让 USB MSC 独占 eMMC */
    if (logMgr_fileOpen)
    {
        f_sync(&logMgr_file);
        f_close(&logMgr_file);
        logMgr_fileOpen = 0;
    }
    f_unmount(logMgr_drivePath);

    log_i("log_mgr: suspended (USB mounted)");
}

void log_mgr_usb_unmounted(void)
{
    /* 重新挂载 FatFS */
    FRESULT res = f_mount(&logMgr_fatfs, logMgr_drivePath, 1);
    if (res != FR_OK)
    {
        log_e("log_mgr: remount failed (%d)", res);
    }

    logMgr_usbActive = 0;
    logMgr_curDay    = 0;   /* 强制下次写入时重新打开文件 */

    log_i("log_mgr: resumed (USB unmounted)");
}
