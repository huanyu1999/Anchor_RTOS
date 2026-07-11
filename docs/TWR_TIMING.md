# TWR 时序统一计算（Phase B，TREK1000 单轨方案）

> 本文是 **Anchor(本工程, RTOS) 与 Tag(裸机工程 `../Tag`) 双端时序对齐的唯一依据**。
> Anchor 端所有 TWR 时序由 `twr_set_replydelay()`（`dw_main.c`，移植 TREK1000
> `instance_set_replydelay()`，`Reference Example\Trek1000-RTLS-master\trek1000\src\application\instance_common.c`）
> 按帧长公式统一算出，装填到 `instance_data_t.timings`（`twrTimings_t`）。
> **旧的 `FIRST_RESP_SEND_*` 等按速率展开的时序宏与 `TWR_TIMING_LEGACY` 双轨开关已于
> 2026-07-09 彻底删除**，空口时序与旧版 Tag 不兼容，Tag 工程必须按本文适配后联调。

## 1. TREK 时序模型（与旧方案的三个本质区别）

1. **统一槽间隔**：只有一个 `fixedReplyDelay`（= resp 整帧时长 + `RX_RESPONSE_TURNAROUND`）。
   首个 resp 槽 = poll RX + 1×，相邻槽 +1×。不再有 firstRespDly/interval 之分，也没有
   ancRespTxBack/finalTxBack 再延后偏移（翻转量已含在槽间隔内）。
2. **RX 开窗提前一个前导码**：DW 延迟发送编程的是 RMARKER（SFD 结束）时刻，延迟接收编程的
   是接收机开机时刻，而前导码在 RMARKER 之前飞完。所以接收方开窗 = 对方 RMARKER 槽时刻 −
   `preambleDuration32h`（TREK `anch_enable_rx()` 同款）。
3. **接收超时用 symbol 单位**（1.0256us，`dwt_setrxtimeout` 原生单位），由帧长公式算出。

时间基：调度量统一用 **32h**（40bit DW 设备时间 >>8，≈4.006ns/单位），TREK
`delayedTRXTime32h` 同款；TOF 计算仍用完整 40bit 硬件时间戳。

## 2. 常量与字段

`dw_instance.h`：

| 常量 | 值 | 含义 |
|---|---|---|
| `DW_RX_ON_DELAY` | 16 us | DW 接收机使能到可收数据的开机延时 |
| `RX_RESPONSE_TURNAROUND` | **500 us** | 帧间处理翻转余量。TREK 裸机原值 300；RTOS 回调路径保守取 500，SWO 实测后收紧（唯一可调余量） |

`twrTimings_t`（`twr_set_replydelay()` 装填）：

| 字段 | 公式 | 消费点 |
|---|---|---|
| `fixedReplyDelayAnc32h` | devtime(preamble + resp数据段 + TURNAROUND) >> 8 | 槽推进 / A2A 槽位 / final 槽位 |
| `preambleDuration32h` | devtime(preamble) >> 8 + DW_RX_ON_DELAY | 所有 delayed RX 开窗提前量 |
| `pollTx2FinalTxDelay32h` | (N+1) × fixedReplyDelayAnc32h | T2A final 接收窗基准 |
| `fixedReplyDelay_sy` | (resp整帧 + TURNAROUND) / 1.0256 | 多槽接收超时窗算术（A2A 容错） |
| `fwto4RespFrame_sy` | DW_RX_ON_DELAY + (preamble + (resp数据段+3000ns)/1000) / 1.0256 | resp 接收超时 |
| `fwto4FinalFrame_sy` | 同式(T2A final) + 200 | final 接收超时（T2A 用 ×2 余量，TREK 同款） |

帧长输入：`calc_length_data(MSG_LEN + FCS_LEN)`（RS(63,55) 编码 + PHR 的 air time，ns；
FCS 也在空口飞，须计入）。preamble = `(plen + sfdlen) × 1.01763us`（PRF64；PRF16 用 0.99359）。

## 3. 时间轴（N = `MAX_AHCHOR_NUMBER`）

```
T2A（槽 k 归基站 k，k=0..N-1）：
poll RMARKER ──1×fixed──► 槽0 ──1×fixed──► 槽1 ──1×fixed──► 槽2 ...
resp_k RMARKER = pollRx + (k+1)×fixedReplyDelay
接收方开窗 = 槽时刻 − preambleDuration
final：Tag final TX RMARKER = poll TX + (N+1)×fixedReplyDelay（双端同公式）
       ── 取 (N+1)× 而非 N×：末槽 resp 数据段在其 RMARKER 后还要飞、final 前导码在其
          RMARKER 前就开始飞，(N+1)× 天然隔开一个槽避免空口重叠
       Anchor final 窗 = pollRx + (N+1)×fixedReplyDelay − preambleDuration，超时 fwto4Final×2

A2A（首个 responder 槽整体后移一槽，调度余量）：
resp2(pos)  RMARKER = pollRx + (pos+2)×fixedReplyDelay   （pos = anc_id − initiator_id − 1）
final       RMARKER = pollTx + (A2A_FINAL_SCHEDULE_INDEX+1)×fixedReplyDelay
                    = pollTx + (N+2)×fixedReplyDelay
```

## 4. 参考数值（850K / 前导1024 / N=3 / TURNAROUND=500，boot log 核对用）

- preamble ≈ (1024+16)×1.01763 ≈ **1058 us**
- resp 数据段(19+2 B) ≈ 243 us → resp 整帧 ≈ 1301 us
- **fixedReplyDelay ≈ 1801 us**（32h ≈ 449,000 量级）
- **pollTx2FinalTxDelay = 4×1801 ≈ 7204 us**
- fwto4RespFrame ≈ 1289 sy；fwto4FinalFrame ≈ 1720 sy
- T2A：末槽 resp 落地于 pollRx+5646us，final 前导码 7204−1058=6146us 才开始 ✓ 无重叠；
  单次交换 ≈ 8.8ms < 12ms slot ✓
- A2A：final RMARKER = pollTx + 5×1801 ≈ 9005us，+final 数据段 ≈ 9.5ms < 12ms slot（偏紧，
  boot log 越界 log_e 兜底；靠收紧 RX_RESPONSE_TURNAROUND 优化）

boot log 输出：`TWR timing: replyDelay=... preamble=... fwtoResp=... fwtoFinal=... pollTx2Final=...`，
另有 log_e 检查：T2A/A2A 单次交换超 slot 时告警。
（`sfConfig.pollTxToFinalTxDly_us` 仅作记录不再被消费，final 时刻由公式导出。）

## 5. Tag 工程适配清单（双端必须一致）

1. **resp 槽位**：Tag 各基站 resp 接收窗按 `pollTx + (k+1)×fixedReplyDelay − preamble` 开，
   `fixedReplyDelay` 用同一公式（同帧长、同 RX_RESPONSE_TURNAROUND=500）算出；
2. **final TX** = poll TX + `(N+1)×fixedReplyDelay`（同公式导出，当前 ≈7204us）；
3. 帧长定义 `POLL_MSG_LEN / RESP_MSG_LEN / FIANL_MSG_LEN`、`FCS_LEN` 计入、`MAX_AHCHOR_NUMBER`；
4. RF 配置（信道/速率/前导码长度/SFD 类型/PRF）；
5. 超帧参数 `ONE_SLOT_TIME_MS_* × inst_slot_number`（A0 sleep correction 基准，未变）。

**联调流程**：Tag 按本文实现同一公式并刷机 →（双端已无 LEGACY 开关，Anchor 先刷不影响 A2A，
但 T2A 在 Tag 刷机前测不了距）→ SWO 实测回调耗时，收紧 `RX_RESPONSE_TURNAROUND` →
回归（成功率、距离、长跑）。
