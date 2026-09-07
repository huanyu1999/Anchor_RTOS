# 四基站 A2A 与 Tag ToF 终端解算方案

**日期：2026-08-07**

## 1. 目标与边界

本方案面向四台 Anchor 部署在矩形四个角的场景：

- A0 是 TCP 汇聚节点，也是 A2A 唯一发起节点；
- A1、A2 分别位于 A0 的相邻角；A3 位于 A0 的对角；
- Tag 的二维坐标由解算终端计算，Anchor 不在 MCU 上做坐标解算或滤波；
- A2A 仍安排在每个超级帧末尾，不增加六条边的完整轮询；
- ISO 0xC5 Blink / RNG_INIT Discovery 默认不开放。

系统对外输出原始 DS-TWR ToF（DTU），而非最终距离。终端负责 ToF 到距离的转换、各 Anchor 的偏差模型、定位、滤波和报警业务。

## 2. A2A 矩形自检

每个超级帧由 A0 发起一轮 A2A，测得三条径向链路：

```
A2 ----- A3
|         |
|         |
A0 ----- A1
```

- `d01 = A0-A1`：矩形长；
- `d02 = A0-A2`：矩形宽；
- `d03 = A0-A3`：矩形对角线。

终端将三条 ToF 换算距离，计算矩形一致性残差：

```
residual = d03^2 - d01^2 - d02^2
```

该方式能验证四角部署是否接近直角矩形，并观察相对测距漂移；不能仅凭三条测量单独解出四块板各自的绝对天线延时。若需要绝对天线延时标定，仍需已知物理距离或测量六条边后进行受约束拟合。

### A2A 控制

- A0 上电后默认开启 A2A；
- 终端可通过 TCP 临时关闭/开启 A2A；
- 关闭命令只阻止下一次超级帧发起，不中断已经开始的三帧 DS-TWR；
- A1/A2/A3 无需 TCP 控制，始终监听 A0 的 A2A POLL。

## 3. Tag ToF 聚合

Tag 的一次 TWR 中，四台 Anchor 都只能在收到 FINAL 后计算本轮 ToF。因此本轮 Anchor RESP 无法携带本轮 ToF，只能携带**上一轮**结果。

时序如下：

1. 第 `n` 轮完成后，每台 Anchor 缓存本机 `tof_dtu`、Tag POLL RSSI、`range_nb=n` 和有效性；
2. 第 `n+1` 轮，各 Anchor 在 RESP 中广播自己第 `n` 轮的缓存；
3. A0 旁听 A1/A2/A3 的 RESP，并将其与 A0 第 `n` 轮缓存按 `range_nb` 对齐；
4. A0 将同一序号的 3 或 4 条 ToF 通过 TCP 推送给终端。

因此定位数据天然比空口测量晚一轮，但不会混合不同测距序号的数据。

### 有效性规则

- 4 条有效 ToF：发送完整组，终端常规四基站解算；
- 3 条有效 ToF：发送降级组，终端依据有效位掩码进行三基站二维解算；
- 少于 3 条：不发送定位数据包，只累加缺站/丢轮统计；
- 所有 ToF 都必须具有相同 `tag_id`、`range_nb`，否则整组丢弃。

## 4. 空口协议调整

### Tag RESP 扩展

现有 RESP 的“上一轮距离”字段改为上一轮原始 ToF，并在尾部追加结果序号与 RSSI：

| 字段 | 长度 | 字节序 | 含义 |
|---|---:|---|---|
| `prev_tof_dtu` | 4 B | 小端 | 上一轮 DS-TWR ToF，`int32_t` DTU；`<=0` 无效 |
| `result_range_nb` | 1 B | - | `prev_tof_dtu` 所属 Tag 测距序号 |
| `rssi_cdbm` | 2 B | 小端 | 接收该轮 Tag POLL 的 RSSI，单位 0.01 dBm；`0x7FFF` 未知 |

Anchor RESP 总长度由 19 B 改为 22 B（不含 FCS）。Tag 工程必须同步更新：RESP 长度、结果字段解释及基于帧长的 TWR 时序计算。旧 Tag 将不能与新 Anchor 固件正确联调。

ToF 是 DS-TWR 公式的 DTU 结果，不执行米制换算，也不应用 DW1000 的 range-bias 表。终端应基于实际链路的芯片类型、天线延时和标定策略统一处理。

### Tag 电量预留

TCP 数据包预留 `tag_battery_mv` 字段。当前未规定 Tag POLL 的电量空口字段时，Anchor 填 `0xFFFF`。后续将电量加入 Tag POLL 后，只需在对应轮次缓存/置有效位，不改变 TCP 帧格式。

## 5. TCP 二进制协议

沿用现有公共帧格式：

```text
[0xAA][0x55][CMD][SEQ][LEN_LE:2][DATA][CRC8]
```

CRC8 覆盖 `CMD` 至 `DATA`，不覆盖 SOF 与 CRC 自身。

### 控制命令

| CMD | 名称 | 方向 | DATA |
|---:|---|---|---|
| `0x21` | `CMD_SET_A2A_ENABLE` | 终端 -> A0 | `enabled:u8`，仅 `0` 或 `1` |
| `0x22` | `CMD_QUERY_A2A_STATUS` | 终端 -> A0 | 空 |

`CMD_SET_A2A_ENABLE` 应答为 `[status:u8][enabled:u8]`。非 A0 接收该命令时返回 `RESP_ERR_ROLE`。

`CMD_QUERY_A2A_STATUS` 应答包含当前开关、三条径向 ToF 的有效位、最近 A2A 序号和缺站统计。

### ToF 推送

| CMD | 名称 | 方向 | 说明 |
|---:|---|---|---|
| `0x30` | `CMD_PUSH_TAG_TOF` | A0 -> 终端 | Tag 的三/四 Anchor ToF 定位组 |
| `0x31` | `CMD_PUSH_A2A_TOF` | A0 -> 终端 | A0-A1/A2/A3 A2A 自检组 |

两种推送使用同一固定 DATA 布局：

```text
version:u8                 // 当前固定为 1
tag_id:u8                  // A2A 推送固定为 0xFF
range_nb:u8
valid_mask:u8              // bit i = Anchor i 的 ToF 有效
flags:u8                   // bit0: 完整四 Anchor；bit1: 三 Anchor 降级
tick_ms:u32                // A0 聚合完成时 tick，little-endian
tag_battery_mv:u16         // 0xFFFF = 未上报，little-endian
tof_dtu[4]:i32             // Anchor 0..3，little-endian
rssi_cdbm[4]:i16           // Anchor 0..3，little-endian；0x7FFF = 未知
```

A2A 只使用 `tof_dtu[1..3]`，`valid_mask` 对应 A1/A2/A3；RSSI 与电量填保留值。

## 6. 任务与并发约束

- `task_uwb` 是时序关键路径，只计算 ToF、更新缓存和非阻塞投递 ToF 快照；
- 不允许在 UWB 回调中调用 W5500 `send()`、文件系统或阻塞 API；
- `task_eth` 从专用消息队列读取快照，编码并通过 TCP 推送；
- 队列满时丢弃最旧之外的新定位快照并递增统计，绝不阻塞 UWB 状态机；
- A2A 与 Tag TWR 仍复用现有互斥状态机，A2A 只在 A0 处于 `ANCHOR + LISTENER` 时发起。

## 7. Discovery 开关

新增编译期开关：

```c
#define UWB_DISCOVERY_ENABLE 0
```

关闭时：

- 不设置 `DWT_FF_RSVD_EN`；
- 收到 reserved/Blink 帧不会进入 Blink/RNG_INIT/tagList 逻辑；
- A0 与其它 Anchor 使用相同的正常 Data/ACK 帧过滤；
- Discovery 源码保留，后续可独立开启并验证，不与本次 ToF/A2A 需求耦合。

## 8. 验收清单

1. DW1000、DW3000 双目标均能构建；Tag 使用同步后的 RESP 长度与字段。
2. 四基站矩形部署时，A0 每超级帧输出 A0-A1/A2/A3 ToF；终端残差稳定且符合现场安装精度。
3. TCP 可关闭/重新开启 A2A，切换不打断正在进行的 TWR 交换。
4. 单 Tag、多 Tag、`range_nb` 从 255 回绕至 0、RESP 丢失和单 Anchor 离线时，A0 不混合不同序号的数据。
5. 四条与三条有效 ToF 分别产生完整/降级定位推送；少于三条不产生定位推送。
6. T2A 与 A2A 长跑期间，未开启 Discovery 时不会处理 Blink 帧或停收。
