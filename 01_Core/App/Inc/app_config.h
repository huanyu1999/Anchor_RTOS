#ifndef __APP_CONFIG_H
#define __APP_CONFIG_H

/**
 * @file  app_config.h
 * @brief 功能模块编译开关 — 置 0 可裁剪对应模块，节省 Flash/RAM
 *
 * 每个宏控制一组相关的：初始化、FreeRTOS 任务、中断回调。
 * 默认全部开启（1），按需关闭（0）。
 */

/* ======================== 核心功能 ======================== */
#define MODULE_UWB_ENABLE           1   /* UWB 测距（DW1000/DW3000） */
#define MODULE_ALARM_ENABLE         0   /* 分级报警（语音 + 蜂鸣器 + LED） */
#define MODULE_CAN_ENABLE           0   /* CAN 总线通信（双基站联动） */

/* ======================== 存储 & 日志 ======================== */
#define MODULE_EMMC_ENABLE          0   /* eMMC + FatFS */
#define MODULE_LOG_MANAGE_ENABLE    0   /* 业务日志写入 eMMC */
#define MODULE_USB_MSC_ENABLE       0   /* USB MSC（U盘模式导出日志） */

/* ======================== 网络 & 卫星授时 ======================== */
#define MODULE_W5500_ENABLE         1   /* W5500 以太网 */
#define MODULE_GNSS_ENABLE          1   /* GNSS 定位 */

/* ======================== 依赖关系检查 ======================== */
#if MODULE_LOG_MANAGE_ENABLE && !MODULE_EMMC_ENABLE
    #error "MODULE_LOG_MANAGE requires MODULE_EMMC"
#endif

#if MODULE_USB_MSC_ENABLE && !MODULE_EMMC_ENABLE
    #error "MODULE_USB_MSC requires MODULE_EMMC"
#endif

#if MODULE_ALARM_ENABLE && !MODULE_UWB_ENABLE
    #error "MODULE_ALARM requires MODULE_UWB"
#endif

#endif /* __APP_CONFIG_H */
