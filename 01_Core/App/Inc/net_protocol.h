#ifndef __NET_PROTOCOL_H__
#define __NET_PROTOCOL_H__

#include <stdint.h>
#include <stddef.h>

/**
 * @brief MCU ↔ 上位机 TCP 二进制通信协议
 *
 * 帧格式：
 *   [SOF1: 0xAA] [SOF2: 0x55] [CMD: 1B] [SEQ: 1B] [LEN: 2B LE] [DATA: LEN B] [CRC8: 1B]
 *
 * - SOF  : 帧头标识
 * - CMD  : 命令字（请求）/ 命令字 | 0x80（应答）
 * - SEQ  : 序号，应答须回填请求的 SEQ
 * - LEN  : DATA 段字节数（little-endian）
 * - DATA : 载荷
 * - CRC8 : 从 CMD 到 DATA 末尾（不含 SOF、不含 CRC 自身）
 */

/* ========================= 帧常量 ========================= */
#define NET_SOF1                0xAA
#define NET_SOF2                0x55
#define NET_FRAME_HEAD_SIZE     6       /* SOF1 + SOF2 + CMD + SEQ + LEN(2) */
#define NET_FRAME_CRC_SIZE      1
#define NET_FRAME_OVERHEAD      (NET_FRAME_HEAD_SIZE + NET_FRAME_CRC_SIZE)  /* 7 */
#define NET_FRAME_MAX_DATA      512
#define NET_FRAME_MAX_SIZE      (NET_FRAME_OVERHEAD + NET_FRAME_MAX_DATA)   /* 519 */

#define NET_RESP_BIT            0x80    /* 应答帧 CMD 最高位置 1 */

/* ========================= 命令字 ========================= */
#define CMD_QUERY_DEV_INFO      0x01    /* 查询设备信息 */
#define CMD_QUERY_CONFIG        0x02    /* 查询当前 RF 配置 */
#define CMD_SET_CONFIG          0x03    /* 设置 RF 参数 */
#define CMD_LOG_LIST            0x10    /* 列举日志目录 */
#define CMD_LOG_READ            0x11    /* 读取日志文件（分块） */
#define CMD_TAG_STATUS          0x20    /* MCU → 上位机：标签实时状态推送 */

/* ========================= 通用应答状态码 ========================= */
#define RESP_OK                 0x00
#define RESP_ERR_UNKNOWN_CMD    0x01
#define RESP_ERR_CRC            0x02
#define RESP_ERR_PARAM          0x03
#define RESP_ERR_BUSY           0x04
#define RESP_ERR_FILE           0x05    /* 文件操作失败 */

/* ========================= 数据结构 ========================= */

/** 单个标签状态（推送 CMD_TAG_STATUS 的子条目） */
typedef struct __attribute__((packed))
{
    uint8_t  tag_id;
    int32_t  distance_mm;       /* 最近一次测距值，单位 mm */
    float    rx_power_dbm;      /* 接收信号强度 */
    uint16_t age_ms;            /* 距上次 TWR 成功的时间，>阈值视为断联 */
} net_tag_entry_t;

/** RF 配置参数（CMD_QUERY_CONFIG / CMD_SET_CONFIG 的 DATA） */
typedef struct __attribute__((packed))
{
    uint8_t  channel;           /* UWB 信道号 */
    uint8_t  dataRate;          /* 数据速率 0=110K,1=850K,2=6.8M */
    uint8_t  prf;               /* PRF 0=16M,1=64M */
    uint32_t tx_power;          /* 发射功率寄存器值 */
    uint16_t ant_delay;         /* 天线延时 */
    int32_t  dist_offset_cm;    /* 距离校准偏移 cm */
} net_rf_config_t;

/** 日志读取请求（CMD_LOG_READ 的 DATA） */
typedef struct __attribute__((packed))
{
    char     filename[32];      /* 相对于 LOG 根目录的路径 */
    uint32_t offset;            /* 文件内偏移 */
    uint16_t length;            /* 请求读取长度，最大 NET_FRAME_MAX_DATA - 6 */
} net_log_read_req_t;

/** 日志读取应答（CMD_LOG_READ 应答的 DATA） */
typedef struct __attribute__((packed))
{
    uint32_t offset;            /* 本块起始偏移 */
    uint32_t file_size;         /* 文件总大小 */
    /* 后跟 actual_len 字节日志内容 */
} net_log_read_resp_hdr_t;

/* ========================= 帧解析器 ========================= */

/** 解析器状态 */
typedef enum
{
    PARSE_SOF1 = 0,
    PARSE_SOF2,
    PARSE_CMD,
    PARSE_SEQ,
    PARSE_LEN_LO,
    PARSE_LEN_HI,
    PARSE_DATA,
    PARSE_CRC
} net_parse_state_t;

/** 解析出的完整帧 */
typedef struct
{
    uint8_t  cmd;
    uint8_t  seq;
    uint16_t data_len;
    uint8_t  data[NET_FRAME_MAX_DATA];
} net_frame_t;

/** 流式帧解析器上下文 */
typedef struct
{
    net_parse_state_t state;
    uint16_t          data_idx;
    net_frame_t       frame;
} net_parser_t;

/* ========================= API ========================= */

/**
 * @brief  初始化协议模块
 */
void net_protocol_init(void);

/**
 * @brief  向帧解析器喂入一段原始字节流
 * @param  parser  解析器上下文
 * @param  buf     输入数据
 * @param  len     输入长度
 * @return 解析出完整帧的数量（0 或 1+），帧内容在 parser->frame
 * @note   每解析出一帧立即调用命令分发，因此返回值主要用于调试
 */
int net_protocol_feed(net_parser_t *parser, const uint8_t *buf, uint16_t len);

/**
 * @brief  打包并发送应答帧
 * @param  cmd      原始请求的 CMD（函数内部自动 | NET_RESP_BIT）
 * @param  seq      与请求相同的 SEQ
 * @param  data     应答载荷，可为 NULL
 * @param  data_len 载荷长度
 * @return 0 成功，-1 发送失败
 */
int net_protocol_send_resp(uint8_t cmd, uint8_t seq, const uint8_t *data, uint16_t data_len);

/**
 * @brief  打包并发送主动推送帧（不带 RESP_BIT）
 * @param  cmd      命令字
 * @param  data     载荷
 * @param  data_len 载荷长度
 * @return 0 成功，-1 发送失败
 */
int net_protocol_send_push(uint8_t cmd, const uint8_t *data, uint16_t data_len);

/**
 * @brief  CRC8 计算 (poly = 0x07, init = 0x00)
 */
uint8_t net_crc8(const uint8_t *data, uint16_t len);

/**
 * @brief  CRC8 续算（在已有 crc 基础上继续计算）
 */
uint8_t net_crc8_continue(uint8_t crc, const uint8_t *data, uint16_t len);

#endif /* __NET_PROTOCOL_H__ */
