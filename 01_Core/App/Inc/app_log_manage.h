#ifndef __APP_LOG_MANAGE_H
#define __APP_LOG_MANAGE_H

#include <stdint.h>

/**
 * @brief 业务日志管理模块
 *
 * 与 EasyLogger (debug SWO) 独立，专用于 eMMC 文件记录。
 *
 * 文件结构：0:/LOG/YYMM/DDA{id}.LOG
 *   YYMM  — 年月目录（如 2603 = 2026年3月）
 *   DDA{id} — 日期 + 基站编号（如 11A01.LOG = 11日基站01）
 *
 * 日志行格式：
 *   [YYYY-MM-DD] [HH:MM:SS] [TYPE] content\r\n
 *
 * 记录内容包括：
 *   - 基站开机
 *   - 语音暂停 / 再次开启
 *   - CAN 通信故障 / 恢复
 *   - 最近报警标签ID、信号强度、报警来源（本地/CAN）
 *   - 系统参数（音量、UWB 发射功率、标签响应时长等）
 */

/* 日志分类 */
typedef enum {
    LOG_MGR_SYS = 0,    /* 系统：开机、音量变更、UWB 功率、标签响应时长 */
    LOG_MGR_ALM,        /* 报警：语音开启/暂停、报警标签、报警来源 */
    LOG_MGR_COM,        /* 通信：CAN 故障/恢复 */
} log_mgr_type_t;

/* 报警来源 */
typedef enum {
    ALARM_SRC_LOCAL = 0, /* 本基站 UWB 测距触发 */
    ALARM_SRC_CAN,       /* 通过 CAN 总线接收自其他基站 */
} alarm_source_t;

/* 无关标签时传入此值 */
#define LOG_MGR_NO_TAG  0xFF

/**
 * @brief  初始化日志管理模块
 *         挂载 FatFS、创建目录、检查容量、启动写入任务
 * @return 0 成功，-1 失败
 */
int log_mgr_init(void);

/**
 * @brief  写入一条业务日志（线程安全，通过队列异步写入）
 * @param  type    日志分类
 * @param  tag_id  相关标签ID（0x00-0xFE），LOG_MGR_NO_TAG 表示无关标签
 * @param  fmt     printf 风格格式串
 */
void log_mgr_write(log_mgr_type_t type, uint8_t tag_id, const char *fmt, ...);

/**
 * @brief  通知日志模块 USB MSC 已挂载，暂停写入并卸载 FatFS
 */
void log_mgr_usb_mounted(void);

/**
 * @brief  通知日志模块 USB MSC 已卸载，重新挂载 FatFS 并恢复写入
 */
void log_mgr_usb_unmounted(void);

#endif /* __APP_LOG_MANAGE_H */
