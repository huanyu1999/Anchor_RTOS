/**
 * @file  net_protocol.c
 * @brief MCU ↔ 上位机 TCP 二进制通信协议实现
 */

#include "net_protocol.h"

#include <string.h>
#include <stdio.h>

#include "elog.h"
#include "ff.h"
#include "socket.h"

#include "dw_instance.h"
#include "dw_sort.h"
#include "dev_w5500.h"

/* ========================= 外部依赖 ========================= */
extern w5500_device dev_w5500;

/* ========================= 内部变量 ========================= */
static uint8_t tx_buf[NET_FRAME_MAX_SIZE];  /* 发送帧缓冲 */
static uint8_t push_seq;                    /* 推送帧自增序号 */

/* ========================= CRC8 ========================= */
uint8_t net_crc8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint8_t net_crc8_continue(uint8_t crc, const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x07;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/* ========================= 帧打包 ========================= */
static int frame_pack_and_send(uint8_t cmd, uint8_t seq,
                               const uint8_t *data, uint16_t data_len)
{
    if (data_len > NET_FRAME_MAX_DATA)
    {
        return -1;
    }

    uint16_t idx = 0;
    tx_buf[idx++] = NET_SOF1;
    tx_buf[idx++] = NET_SOF2;
    tx_buf[idx++] = cmd;
    tx_buf[idx++] = seq;
    tx_buf[idx++] = (uint8_t)(data_len & 0xFF);
    tx_buf[idx++] = (uint8_t)((data_len >> 8) & 0xFF);

    if (data && data_len > 0)
    {
        memcpy(&tx_buf[idx], data, data_len);
        idx += data_len;
    }

    /* CRC8 覆盖 CMD ~ DATA */
    uint8_t crc = net_crc8(&tx_buf[2], 4 + data_len);
    tx_buf[idx++] = crc;

    /* 通过 W5500 socket 0 发送 */
    int32_t ret = send(SOCKET_ID, tx_buf, idx);
    if (ret < 0)
    {
        log_e("net_protocol send failed: %d", ret);
        return -1;
    }
    return 0;
}

int net_protocol_send_resp(uint8_t cmd, uint8_t seq,
                           const uint8_t *data, uint16_t data_len)
{
    return frame_pack_and_send(cmd | NET_RESP_BIT, seq, data, data_len);
}

int net_protocol_send_push(uint8_t cmd, const uint8_t *data, uint16_t data_len)
{
    return frame_pack_and_send(cmd, push_seq++, data, data_len);
}

/* ========================= 命令处理 ========================= */

/** CMD_QUERY_DEV_INFO 应答：设备信息 */
static void handle_query_dev_info(const net_frame_t *req)
{
    /* 载荷: [anc_id 1B] [version_str NB] */
    uint8_t resp[64];
    uint16_t idx = 0;

    resp[idx++] = RESP_OK;
    resp[idx++] = anc_id;

    const char *ver = SOFTWARE_VER;
    uint8_t ver_len = (uint8_t)strlen(ver);
    resp[idx++] = ver_len;
    memcpy(&resp[idx], ver, ver_len);
    idx += ver_len;

    net_protocol_send_resp(req->cmd, req->seq, resp, idx);
}

/** CMD_QUERY_CONFIG 应答：当前 RF 配置 */
static void handle_query_config(const net_frame_t *req)
{
    uint8_t resp[1 + sizeof(net_rf_config_t)];
    resp[0] = RESP_OK;

    net_rf_config_t *cfg = (net_rf_config_t *)&resp[1];
    cfg->channel       = inst_ch;
    cfg->dataRate      = inst_dataRate;
    cfg->prf           = inst_prf;
    cfg->tx_power      = TX_POWER;
    cfg->ant_delay     = ant_dly;
    cfg->dist_offset_cm = distance_offset_cm;

    net_protocol_send_resp(req->cmd, req->seq, resp, sizeof(resp));
}

/** CMD_SET_CONFIG 处理：设置 RF 参数 */
static void handle_set_config(const net_frame_t *req)
{
    if (req->data_len < sizeof(net_rf_config_t))
    {
        uint8_t resp = RESP_ERR_PARAM;
        net_protocol_send_resp(req->cmd, req->seq, &resp, 1);
        return;
    }

    const net_rf_config_t *cfg = (const net_rf_config_t *)req->data;

    /* 应用配置到全局变量（运行时生效，不持久化） */
    inst_ch           = cfg->channel;
    inst_dataRate     = cfg->dataRate;
    inst_prf          = cfg->prf;
    /* tx_power 需通过 DW1000 API 配置，此处仅记录 */
    ant_dly           = cfg->ant_delay;
    distance_offset_cm = cfg->dist_offset_cm;

    log_i("RF config updated: ch=%d rate=%d prf=%d ant_dly=%d offset=%d",
          inst_ch, inst_dataRate, inst_prf, ant_dly, distance_offset_cm);

    uint8_t resp = RESP_OK;
    net_protocol_send_resp(req->cmd, req->seq, &resp, 1);
}

/** CMD_LOG_LIST 处理：列举日志目录 */
static void handle_log_list(const net_frame_t *req)
{
    DIR dir;
    FILINFO fno;
    FRESULT res;
    uint8_t resp[NET_FRAME_MAX_DATA];
    uint16_t idx = 0;

    resp[idx++] = RESP_OK;

    /* 打开 LOG 根目录 */
    res = f_opendir(&dir, "0:/LOG");
    if (res != FR_OK)
    {
        resp[0] = RESP_ERR_FILE;
        net_protocol_send_resp(req->cmd, req->seq, resp, 1);
        return;
    }

    /* 遍历子目录和文件，以 '\n' 分隔 */
    while (idx < NET_FRAME_MAX_DATA - 48)
    {
        res = f_readdir(&dir, &fno);
        if (res != FR_OK || fno.fname[0] == 0)
        {
            break;
        }
        uint8_t name_len = (uint8_t)strlen(fno.fname);
        if (idx + name_len + 2 > NET_FRAME_MAX_DATA)
        {
            break;
        }
        /* 目录标记 'D' / 文件标记 'F' */
        resp[idx++] = (fno.fattrib & AM_DIR) ? 'D' : 'F';
        memcpy(&resp[idx], fno.fname, name_len);
        idx += name_len;
        resp[idx++] = '\n';
    }

    f_closedir(&dir);
    net_protocol_send_resp(req->cmd, req->seq, resp, idx);
}

/** CMD_LOG_READ 处理：读取日志文件块 */
static void handle_log_read(const net_frame_t *req)
{
    if (req->data_len < sizeof(net_log_read_req_t))
    {
        uint8_t resp = RESP_ERR_PARAM;
        net_protocol_send_resp(req->cmd, req->seq, &resp, 1);
        return;
    }

    const net_log_read_req_t *rr = (const net_log_read_req_t *)req->data;

    /* 构造完整路径 */
    char path[64];
    snprintf(path, sizeof(path), "0:/LOG/%s", rr->filename);

    FIL fil;
    FRESULT res = f_open(&fil, path, FA_READ);
    if (res != FR_OK)
    {
        uint8_t resp = RESP_ERR_FILE;
        net_protocol_send_resp(req->cmd, req->seq, &resp, 1);
        return;
    }

    uint32_t file_size = f_size(&fil);

    /* 限制单次读取长度 */
    uint16_t read_len = rr->length;
    uint16_t max_chunk = NET_FRAME_MAX_DATA - 1 - sizeof(net_log_read_resp_hdr_t);
    if (read_len > max_chunk)
    {
        read_len = max_chunk;
    }

    uint8_t resp[NET_FRAME_MAX_DATA];
    uint16_t idx = 0;
    resp[idx++] = RESP_OK;

    net_log_read_resp_hdr_t *hdr = (net_log_read_resp_hdr_t *)&resp[idx];
    hdr->offset    = rr->offset;
    hdr->file_size = file_size;
    idx += sizeof(net_log_read_resp_hdr_t);

    /* seek & read */
    UINT br = 0;
    if (rr->offset < file_size)
    {
        f_lseek(&fil, rr->offset);
        f_read(&fil, &resp[idx], read_len, &br);
    }
    idx += br;

    f_close(&fil);
    net_protocol_send_resp(req->cmd, req->seq, resp, idx);
}

/** 命令分发 */
static void dispatch_frame(const net_frame_t *frame)
{
    /* 忽略应答帧（MCU 不处理来自上位机的应答） */
    if (frame->cmd & NET_RESP_BIT)
    {
        return;
    }

    switch (frame->cmd)
    {
    case CMD_QUERY_DEV_INFO:
        handle_query_dev_info(frame);
        break;
    case CMD_QUERY_CONFIG:
        handle_query_config(frame);
        break;
    case CMD_SET_CONFIG:
        handle_set_config(frame);
        break;
    case CMD_LOG_LIST:
        handle_log_list(frame);
        break;
    case CMD_LOG_READ:
        handle_log_read(frame);
        break;
    default:
    {
        uint8_t resp = RESP_ERR_UNKNOWN_CMD;
        net_protocol_send_resp(frame->cmd, frame->seq, &resp, 1);
        break;
    }
    }
}

/* ========================= 帧解析器 ========================= */

void net_protocol_init(void)
{
    push_seq = 0;
    memset(tx_buf, 0, sizeof(tx_buf));
}

int net_protocol_feed(net_parser_t *parser, const uint8_t *buf, uint16_t len)
{
    int frames_parsed = 0;

    for (uint16_t i = 0; i < len; i++)
    {
        uint8_t byte = buf[i];

        switch (parser->state)
        {
        case PARSE_SOF1:
            if (byte == NET_SOF1)
            {
                parser->state = PARSE_SOF2;
            }
            break;

        case PARSE_SOF2:
            if (byte == NET_SOF2)
            {
                parser->state = PARSE_CMD;
            }
            else
            {
                parser->state = PARSE_SOF1;
            }
            break;

        case PARSE_CMD:
            parser->frame.cmd = byte;
            parser->state = PARSE_SEQ;
            break;

        case PARSE_SEQ:
            parser->frame.seq = byte;
            parser->state = PARSE_LEN_LO;
            break;

        case PARSE_LEN_LO:
            parser->frame.data_len = byte;
            parser->state = PARSE_LEN_HI;
            break;

        case PARSE_LEN_HI:
            parser->frame.data_len |= ((uint16_t)byte << 8);
            if (parser->frame.data_len > NET_FRAME_MAX_DATA)
            {
                /* 长度非法，重置 */
                parser->state = PARSE_SOF1;
            }
            else if (parser->frame.data_len == 0)
            {
                parser->state = PARSE_CRC;
            }
            else
            {
                parser->data_idx = 0;
                parser->state = PARSE_DATA;
            }
            break;

        case PARSE_DATA:
            parser->frame.data[parser->data_idx++] = byte;
            if (parser->data_idx >= parser->frame.data_len)
            {
                parser->state = PARSE_CRC;
            }
            break;

        case PARSE_CRC:
        {
            /* 重建 CRC 校验区域：CMD + SEQ + LEN_LO + LEN_HI + DATA */
            uint8_t crc_buf[4];
            crc_buf[0] = parser->frame.cmd;
            crc_buf[1] = parser->frame.seq;
            crc_buf[2] = (uint8_t)(parser->frame.data_len & 0xFF);
            crc_buf[3] = (uint8_t)((parser->frame.data_len >> 8) & 0xFF);

            uint8_t crc = net_crc8(crc_buf, 4);
            if (parser->frame.data_len > 0)
            {
                crc = net_crc8_continue(crc, parser->frame.data, parser->frame.data_len);
            }

            if (crc == byte)
            {
                dispatch_frame(&parser->frame);
                frames_parsed++;
            }
            else
            {
                log_w("net_protocol CRC mismatch: expect 0x%02X got 0x%02X", crc, byte);
            }

            parser->state = PARSE_SOF1;
            break;
        }
        }
    }

    return frames_parsed;
}
