# TWR 时序统一计算（Phase B）

> 本文是 **Anchor(本工程, RTOS) 与 Tag(裸机工程 `../Tag`) 双端时序对齐的唯一依据**。
> Anchor 端所有 TWR 时序自 Phase B 起由 `twr_set_replydelay()`（`dw_main.c`）统一装填到
> `instance_data_t.timings`（`twrTimings_t`），旧的 `FIRST_RESP_SEND_*` 等宏只剩 LEGACY 装填一处引用。

## 1. 模式开关

`dw_instance.h`:

| 宏 | 取值 | 含义 |
|---|---|---|
| `TWR_TIMING_LEGACY` | **1（当前默认）** | 从旧宏装填，空口时序与现网 Tag 完全一致 |
| | 0 | 按 §4 公式计算（更省空口时间）。**必须与 Tag 工程同步适配后同时刷机** |
| `TWR_TURNAROUND_US` | 500 | 公式模式唯一可调余量：帧间处理翻转时间（含 RTOS 调度），SWO 实测后收紧 |

boot log 会同时打印生效组（`TWR timing active`）与公式参考组（`TWR timing calc-ref`），
以及"单次交换总时长 > slot 时长"的越界告警（log_e）。

## 2. 字段与消费点

时间基准：DW 时间戳打在 **RMARKER**（SFD 结束处）；延迟发送编程的也是 RMARKER 时刻，
前导码在该时刻之前发出。

| `twrTimings_t` 字段 | 含义 | 替代的旧符号 |
|---|---|---|
| `firstRespDly_us` | poll RX(RMARKER) → 首个 resp 槽基准 | `FIRST_RESP_SEND_*` |
| `replyInterval_us` | 相邻 resp 槽间隔 | `DATA_INTERVAL_TIME_*` / `inst_data_interval` |
| `ancRespTxBack_us` | 基站 resp TX 在槽基准上的再延后量 | `ANC_RESP_SEND_BACK_*` |
| `finalTxBack_us` | final TX 在槽基准上的再延后量 | `TAG_FINALE_SEND_BACK_*` |
| `respRxTimeout_us` | resp 接收超时（从开窗起算） | `RESP_RX_TIMEOUT_*` / `inst_resp_rx_timeout` |
| `finalRxTimeout_us` | final 接收超时 | `FINAL_RX_TIMEOUT_*` / `inst_final_rx_timeout` |
| `pollRx2FinalRx_dwt` | poll RX → final RX 开窗（dwt 单位）= `(firstRespDly + N×interval)×UUS` | `inst_poll2final_time` |
| `rngInitTxDly_us` | blink RX → RNG_INIT TX（Phase C discovery）| （新增） |

时间轴（N = `MAX_AHCHOR_NUMBER`，槽 k 属于基站 k）：

```
poll RMARKER ──firstRespDly──► 槽0 ──interval──► 槽1 ──interval──► 槽2 ─...─► final RX 开窗
                              │◄ancBack►resp0.TX                            (poll+firstRespDly+N×interval)
基站开窗接收他站 resp 于槽起点；自己的槽在 槽起点+ancRespTxBack 发送。
Tag 端 final TX = poll TX + firstRespDly + N×interval + finalTxBack。
A2A 复用同一组字段（首个 responder 槽整体后移 1 个 interval，final 槽位 = N+1，见 dw_instance_anchor.c）。
```

## 3. LEGACY 模式数值（与现网一致，2026-07 现状）

| 字段 | 850K | 6P8M | 110K(仅 DW1000) |
|---|---|---|---|
| firstRespDly_us | 1300 | 900 | 3000 |
| replyInterval_us | 1600 | 1100 | 3900 |
| ancRespTxBack_us | 300 | 100 | 1080 |
| finalTxBack_us | 300 | 100 | 1080 |
| respRxTimeout_us | 1000 | 450 | 3800 |
| finalRxTimeout_us | 1300 | 600 | 6000 |
| pollRx2FinalRx (N=3) | 6100us | 4200us | 14700us |
| slot 预算（1 slot） | 12ms | 9ms | 28ms |

单次交换占用 ≈ firstRespDly + N×interval + finalRxTimeout = 7400us @850K，须 < slot 时长。

## 4. 公式模式（`TWR_TIMING_LEGACY=0`，未联调）

参数：
- `sym_us = 1.01763`（PRF64 前导符号时长 us；PRF16 用 0.99359）
- `sfdlen`：DW 非标 SFD 按速率 110K=64 / 850K=16 / 6M8=8 symbol
- `plen`：前导码 symbol 数（RF 配置，当前主用 1024）
- `calc_length_data(len)`：RMARKER 之后数据段 air time(ns)，含 RS(63,55) 编码与 PHR
  （110K PHR=172308ns，850K/6M8=21539ns）

公式：

```
preamble_us      = (plen + sfdlen) × sym_us
data_us(len)     = calc_length_data(len) / 1000
firstRespDly_us  = data_us(POLL_MSG_LEN) + TWR_TURNAROUND_US + preamble_us
replyInterval_us = preamble_us + data_us(RESP_MSG_LEN) + TWR_TURNAROUND_US
ancRespTxBack_us = finalTxBack_us = TWR_TURNAROUND_US / 2
respRxTimeout_us = ancRespTxBack + preamble_us + data_us(RESP) + TWR_TURNAROUND_US/2
finalRxTimeout_us= finalTxBack + preamble_us + data_us(FINAL) + TWR_TURNAROUND_US/2
pollRx2FinalRx   = (firstRespDly + N × replyInterval) × UUS_TO_DWT_TIME
```

参考值 @850K/前导1024/N=3：preamble≈1058us，firstRespDly≈1760us，interval≈1785us
（比 LEGACY 宽松，因为 LEGACY 的隐含 turnaround 只有 ~300us；`TWR_TURNAROUND_US`
按 SWO 实测 poll RX 回调进入→starttx 返回的真实耗时收紧后，公式值应能压到 LEGACY 之下）。

## 5. 双端必须一致的量（Tag 工程适配清单）

1. `firstRespDly_us`、`replyInterval_us`（Tag 开 resp 接收窗 / 计算各基站 resp 槽位）；
2. `finalTxBack_us`（Tag final TX 时刻 = poll TX + firstRespDly + N×interval + finalTxBack；
   对应 Anchor 的 `pollRx2FinalRx_dwt` 开窗）；
3. 帧长定义 `POLL_MSG_LEN / RESP_MSG_LEN / FIANL_MSG_LEN` 与 `MAX_AHCHOR_NUMBER`；
4. RF 配置（信道/速率/前导码长度/SFD 类型/PRF）；
5. 超帧参数 `ONE_SLOT_TIME_MS_* × inst_slot_number`（A0 sleep correction 基准）。

**切换流程**：Tag 按本文实现同一公式 → 双端同时置 `TWR_TIMING_LEGACY=0` 并刷机 →
SWO 校准 `TWR_TURNAROUND_US` → 回归（成功率、距离、长跑）。
