# A2A（锚-锚）测距调试记录：从 15m 爆表到 cm 级

> 适用：本项目 DW1000 / DW3000 混合基站的 Anchor-to-Anchor（A2A）TWR 测距
> 关联代码：`01_Core/App/Src/dw_instance_anchor.c`、`01_Core/App/Inc/dw_instance.h`
> 关联文档：天线延时通用原理见 [`antenna_delay_calibration.md`](./antenna_delay_calibration.md)
> 场景记录：A0 = DW1000（发起方），A1 = DW3000（响应方），实际距离 < 1m

本文档记录一次 A2A 测距从「实际 <1m 却测出 15m+」到「cm 级正常」的完整排查，包含两个独立问题的**原理、思路、数值推导**，方便后续复现/回溯/再标定。

---

## 0. 背景：A2A 协议里谁在算距离

A2A 是让基站互相测距（自定位）。三帧 DS-TWR：

```
A0 (INITIATOR/DW1000)          A1 (RESPONDER_A/DW3000)
  |------ POLL --------------------->|   A1 记 poll_rx
  |<----- RESP2 ---------------------|   A0 记 resp_rx，A1 记 resp_tx
  |------ FINAL(带时间戳) ----------->|   A1 记 final_rx → 在此算距离
```

**关键：距离只在响应方（A1）算一次**（`dw_instance_anchor.c` 的 `RTLS_MSG_ANCH_FINAL` 分支）。
A0 自己不算，它是靠下一轮 RESP2 的 `prev_dis` 字段把 A1 算好的结果回传回来的。
→ **所以所有距离计算 / 标定补偿只需在响应方这一处处理，就全局生效。**

### 非对称 DS-TWR 公式

```
        Ra·Rb − Da·Db
tof = ─────────────────      （单位 DTU，1 DTU ≈ 15.65 ps）
      Ra + Rb + Da + Db
distance = tof × c            （c = SPEED_OF_LIGHT = 299702547 m/s）
```

六个时间戳的来源（A1 在收到 FINAL 时凑齐）：

| 量 | 定义 | 来源 | 是否含天线延时 |
|---|---|---|---|
| `poll_tx` | A0 发 POLL | FINAL 帧内（A0 硬件读） | ✅ 自动含 TX 延时 |
| `poll_rx` | A1 收 POLL | 本地 `get_rx_timestamp_u64()` | ✅ 自动含 RX 延时 |
| `resp_tx` | A1 发 RESP2 | 本地 `get_tx_timestamp_u64()` | ✅ 自动含 TX 延时 |
| `resp_rx` | A0 收 RESP2 | FINAL 帧内（A0 硬件读） | ✅ 自动含 RX 延时 |
| `final_tx` | A0 发 FINAL | FINAL 帧内（**A0 手工算的调度时刻**） | ❌ **需手工补 TX 延时** ← Bug #1 |
| `final_rx` | A1 收 FINAL | 本地 `get_rx_timestamp_u64()` | ✅ 自动含 RX 延时 |

```
Ra = resp_rx  − poll_tx     Rb = final_rx − resp_tx
Da = final_tx − resp_rx     Db = resp_tx  − poll_rx
```

---

## 1. Bug #1：FINAL 帧的 final_tx 漏加发射天线延时（15m 爆表）

### 现象
实际 <1m，测出 15m+。误差极大且偏正。

### 根因
`final_tx` 与 `poll_tx`/`resp_tx` 不同：后两者是**硬件时间戳寄存器**读出的，DW 芯片会**自动加上 TX 天线延时**；而 `final_tx` 是延时发送、由**调度时刻手工算出**的，硬件不会替你加，必须手工补 `+ TX_ANT_DLY`。这是 DecaWave DS-TWR 例程里标准的一步（`(final_tx_time & 0xFFFFFFFE00) + TX_ANT_DLY`）。

之前的版本把这个 `+ ant_dly` 当成"重复补偿"删掉了 → `final_tx` 比真实值**小了一个 `ant_dly`（16549 DTU）** → A1 端算出的 `Da = final_tx − resp_rx` 偏小。

### 为什么偏小的 Da 会放大成 15m

对 `Da` 求偏导（`tof ≪ sum` 时）：

```
∂tof/∂Da ≈ −Db / sum
δtof ≈ (∂tof/∂Da)·δDa = (−Db/sum)·(−ε) = ε · Db/sum        ε = 16549 DTU
```

代入 850K、`MAX_AHCHOR_NUMBER=3`、A0+A1 在线的时序常量（`dw_instance.h`）：

```
FIRST_RESP_SEND_850K = 1300 µs
DATA_INTERVAL_TIME_850K = 1600 µs
ANC_RESP_SEND_BACK_850K = 300 µs
TAG_FINALE_SEND_BACK_850K = 300 µs
A2A_FINAL_SCHEDULE_INDEX = (MAX_AHCHOR_NUMBER-1) + 2 = 4
A1 resp_position = anc_id - initiator_id - 1 = 0

Db = resp_tx − poll_rx ≈ 1300 + 1×1600 + 300            = 3200 µs
final_tx 偏移 = 1300 + 4×1600 + 300                      = 8000 µs
Da = final_tx − resp_rx ≈ 8000 − 3200                    = 4800 µs
Ra ≈ 3200 µs   Rb ≈ 4800 µs
sum = 3200 + 4800 + 4800 + 3200                          = 16000 µs
```

```
δtof ≈ 16549 × (3200 / 16000) = 3310 DTU
     = 3310 × 15.65 ps = 51.8 ns
δd  ≈ 51.8 ns × 3×10⁸ m/s ≈ 15.5 m       ✅ 与实测 ~15m 吻合
```

**符号**：`Da` 偏小 → `Da·Db` 偏小 → 分子 `Ra·Rb − Da·Db` 偏大 → `tof` 偏大 → **距离被高估**（真实 1m 读成 15m）。

### 修复
`dw_instance_anchor.c`（`anch_a2a_sendFinal`）：

```c
uint64_t final_tx_ts_embed = (a2a_final_tx_time & MASK_TXDTS) + ant_dly;  // 补回 TX 天线延时
final_msg_set_ts(&tx_anch_final_msg[A2A_FINAL_FINAL_TX_TS_IDX], final_tx_ts_embed);
```

> 注：`MASK_TXDTS = 0x00FFFFFFFE00`（屏蔽低 9 位）正好等于硬件延时发送的时间粒度，所以 `(a2a_final_tx_time & MASK_TXDTS) + ant_dly` = FINAL 实际离开天线的硬件时间戳。

---

## 2. Bug #2：残留 ~0.95m 固定偏差 —— 混芯片必须分芯片标定天线延时

### 现象
修完 Bug #1 后：实际 0.75m → 测 −0.2m；实际 2m → 测 ~1.0m。
**偏差恒定 ≈ −0.95m，不随距离变** → 典型的天线延时固定偏移（`antenna_delay_calibration.md` §4 的判据）。

### 为什么「offset」是错误的层次
最初想到在响应方直接 `dist += offset` 补 0.95m。但：

> **天线延时是「每颗芯片」的物理量，而 offset 是「每一对组合」的量。**

A2A 两端都是基站，可能是 DW1000-DW1000 / DW3000-DW3000 / 混合。offset 只对当前这一对成立，换配对就得重标 → 不可维护。正确做法是把校正落到**每颗芯片自己的 `ANT_DLY`** 上（编译期各自烧进去），DS-TWR 公式会对**任意配对**自动抵消，一次标定永久有效。

### 数值：误差只能测出「两芯片之和」

误差模型（含各自天线延时，见 `antenna_delay_calibration.md` §2）：

```
tof_est = tof_raw − (D_A0 + D_A1)
误差(距离) = −(ΔD_A0 + ΔD_A1) × (c × DWT)      ΔD = 配置值 − 真实值
c × DWT = 299702547 × 15.65 ps ≈ 4.69 mm / DTU
```

```
−0.95 m = −(ΔD_A0 + ΔD_A1) × 4.69 mm
ΔD_A0 + ΔD_A1 = 950 / 4.69 ≈ 202 DTU
```

单次混合对测距**只能得到两者之和 = 202**，无法分离。于是引入一个合理假设：

- `ANT_DLY_DW1000 = 16549` 是先前按标签链路调过、可接受的值 → 当作**基准**，`ΔD_A0(DW1000) ≈ 0`。
- `ANT_DLY_DW3000 = 16549` 是**直接照抄 DW1000 的，从没为 DW3000 单独标过** → 202 全归在它头上。

### 修复
`dw_instance.h`：

```c
#define ANT_DLY_DW1000   16549   // 基准（标签链路已标定）
#define ANT_DLY_DW3000   16347   // 16549 − 202，DW3000 独立标定
```

`ANT_DLY_DW3000 −= 202` → 混合对误差归零；DW3000-DW3000、DW3000-标签 也随之变准（前提：DW1000 确为准基准）。同时**删除**了临时的 `A2A_DIST_OFFSET_MM`。

---

## 3. 灵敏度：4.69 vs 9.38 mm/DTU（别搞混）

调 `ANT_DLY` 时距离变化量，取决于**这次改动动了几个端点的延时**：

| 场景 | 改一个 `ANT_DLY` 影响的端点 | 灵敏度 |
|---|---|---|
| 混合对，只调 DW3000（**当前**） | 1 个（仅 A1） | **4.69 mm / DTU** |
| 同型号对（如 DW3000-DW3000） | 2 个（两端同值同变） | **9.38 mm / DTU** |
| 锚-标签，只调锚 | 1 个（仅锚） | 4.69 mm / DTU |
| 旧代码：两端共用一个全局 `ANT_DLY` | 2 个 | 9.38 mm / DTU |

> `antenna_delay_calibration.md` §3 给的 9.4 mm/单位是"两端同变"的情形（旧的共用 `ANT_DLY`）；本次是混合对只动一颗芯片，所以是它的一半 4.69。二者不矛盾，取决于你的改动移动了几个端点。

**当前微调速算**：混合对若还偏小 X mm，`ANT_DLY_DW3000 −= X/4.69`。

---

## 4. 待验证的假设 & 后续标定

1. **「DW1000=16549 是准基准」这个假设需验证。** 单颗一种芯片时只能测出和，分不开。
   - 有条件：拿**两颗同型号**测一次（DW1000↔DW1000 或 DW3000↔DW3000），偏差 = `2×ΔD`，即可独立标定每颗（灵敏度 9.38 mm/DTU）。
   - 若 DW1000-DW1000 也偏，则 DW1000 也要回调，A2A 的 202 需在两颗间重新分配。
2. **标签端以后有问题，只回调对应芯片的 `ANT_DLY`**，DW1000 / DW3000 互不影响 —— 这正是分芯片标定的好处。
3. 标定务必：已知距离（2~3m，别太近）、多样本看中位数、多个距离交叉验证（恒定偏差才是天线延时）。详见 `antenna_delay_calibration.md` §4。

---

## 5. 改动汇总 & 重新烧录

| 文件 | 改动 | 修的问题 |
|---|---|---|
| `dw_instance_anchor.c`（`anch_a2a_sendFinal`） | `final_tx` 补 `+ ant_dly` | Bug #1：15m 爆表 |
| `dw_instance.h` | `ANT_DLY_DW3000` 16549 → 16347 | Bug #2：~0.95m 固定偏差 |

**两台都要重新编译烧录**：
- A0（DW1000）：Bug #1 的修复在发起方的 `anch_a2a_sendFinal`，A0 是发起方 → 必须重烧。
- A1（DW3000）：要拿到新的 `ANT_DLY_DW3000` → 必须重烧（`USE_DW3000` 配置）。

---

## 6. 一句话回顾

> **Bug #1**：手工算出来的 `final_tx` 时间戳漏了天线延时（硬件读的会自动加，手算的不会）→ `Da` 偏小 → 距离被 `ε·Db/sum ≈ 15m` 放大。
> **Bug #2**：天线延时是每颗芯片的物理量，DW3000 一直照抄 DW1000 没单独标 → 分芯片标定（`ANT_DLY_DW3000 −= 202`），而不是加一个只对当前配对成立的 offset。
