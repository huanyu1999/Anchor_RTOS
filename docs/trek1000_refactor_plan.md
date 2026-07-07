# TREK1000 参考重构方案(4 阶段)

> **目的**:参照 DecaWave TREK1000 官方实现(规格见 `.claude/TREK1000_TWR_STATE_MACHINE_REFERENCE.md`),
> 重构本项目 UWB TWR 基站代码,统一实例管理、事件流、时序计算,消除多年累积的全局变量散落与重复代码。
>
> **本文档是跨电脑工作交接的唯一事实来源**。每完成一个阶段,更新本文档的进度标记并随代码一起提交。
>
> ⚠️ **阶段编号说明**:本文档旧版曾把"结构统一"和"事件流重构"拆成两个阶段(A/B),与批准方案错位一档。
> 2026-07-07 起恢复为批准方案编号:**A=结构统一+回调直驱事件流,B=统一时序计算,C=Discovery,D=清理**。
> 提交 `f816688` 标题中的 "phase B" 指旧编号,实为本文 Phase A 的一部分。

---

## 工作流程约定

1. **每个 Phase 结束必须停下**:由用户本人编译(`cmake --build build`,DW1000/DW3000 双目标)
   并烧录真机验证,通过后才进入下一阶段;编译、烧录均由用户执行;
2. 每阶段独立提交,格式:`参考Trek1000代码进行重构, phase X: <内容> [YYYY-MM-DD HH:MM]`;
3. 阶段内如需中断,提交信息注明断点位置,并同步更新本文档"当前断点"一节。

## 进度总览(2026-07-07 更新)

| 阶段 | 内容 | 状态 | 提交 |
|------|------|------|------|
| Phase A | `instance_data_t` 统一 + 回调直驱事件流 | ✅ 代码完成(待真机回归) | `4542790`+`f816688`(中间态)+本次收尾提交 |
| Phase B | `twr_set_replydelay()` 统一时序计算 | 🔶 进行中 | — |
| Phase C | Discovery 基站侧(ISO 0xC5 blink) | ⬜ 未开始 | — |
| Phase D | 冗余清理 | ⬜ 未开始 | — |

### 当前断点

无代码断点。Phase A 代码全部完成;Phase B 进行中。

---

## 背景:重构前的问题

- TWR 轮转状态散落在 `dwDevice_t` + 十几个文件级全局变量(`frame_seq_nb`、`range_nb`、
  `poll_rx_ts`、`handleResp_times`、A2A 一整套 `a2a_*` 静态变量)中,T2A 与 A2A 各维护一份;
- 事件流走 `ISR → sema_uwbInt → task_uwb → queue_uwbEvent → task_twrRun → onEvent`,
  比 TREK1000 多一次队列转发和上下文切换,挤占延迟发送时间窗;
- 速率相关时序参数(`FIRST_RESP_SEND_*` 等宏)在 T2A/A2A 路径重复展开 6 处 if-else 阶梯,
  TREK1000 是 `instance_set_replydelay()` 一个函数按帧长公式统一算出。

## 已确认决策(与用户对齐)

| 决策点 | 结论 |
|---|---|
| 空口时序变更 | 允许。Tag 工程(`../Tag`,裸机)同步梳理更新;`TWR_TIMING_LEGACY` 开关做过渡 |
| Blink 帧格式 | Trek 原版 ISO 0xC5 EUI-64 blink(12 字节);RNG_INIT 用长目的+短源混合帧头 |
| 延迟架构 | 时间关键路径移入回调(task_uwb 上下文),Trek 同款;task_twrRun 删除 |
| instance_data_t 范围 | 只收内部状态;对外全局保留:`anc_id/group_id/tag_id/distance_report[]/group_report[]/range_status/range_time/rx_power/ant_dly/distance_offset_cm/inst_ch/inst_prf/inst_dataRate` |
| 交换状态 | **T2A/A2A 共用一套**(`remainingRespToRx/rxRespMask/共享时间戳`),禁止 a2a_* 平行副本;TX-done 用 `lastTxFcode` 分流 |

---

## Phase A:`instance_data_t` 统一 + 回调直驱事件流 ✅

**参照**:TREK §1(instance_data_t)、§8(callbacks 内直驱状态机)

### 完成内容

- **`dw_instance.h`**:`dwDevice_t` → `instance_data_t`(单例,`instance_get()`),收编:
  角色(`device_mode/twr_mode/device_id/gatewayAnchor`)、
  轮转(`remainingRespToRx/rxRespMask/wait4final/lastTxFcode/frame_seq_nb/range_nb/
  a2a_range_nb/recv_tag_id/resp_valid/nextSlotTime`)、
  时间戳(responder `poll_rx_ts/resp_tx_ts/final_rx_ts`,initiator `poll_tx_ts/resp_rx_ts[]/
  final_tx_time/final_rx_time`)、`prev_range[]`、ANCRANGE 的 `a2a_distance[]/sframePeriod_ms/a2aStartTime_ms`;
- **`dw_main.c`**:4 回调直接调 `current_Algorithm->onEvent()`(task_uwb 上下文);
  `task_uwb` 循环 = `osSemaphoreAcquire(sema_uwbInt, A0?sframePeriod:forever)` + 尾部
  `anch_checkA2ATrigger()`;`queue_uwbEvent`/`task_twrRun` 删除(消除双任务并发访问 DW SPI 风险);
  `calc_length_data()` 110K 分支加 `USE_DW1000` 保护(修 DW3000 目标编译错);
- **`dw_instance_anchor.c`**:全函数 `inst->` 化,`a2aState_t`/`a2a_*` 静态副本/
  `handleResp_times`/`respTxIndex` 移除;核心机制:
  - **`anch_respSlotProcess()`**(替代 `anch_txRespOrRxReEnable`)统一 resp 槽推进引擎:
    `nextSlotTime` 指向下一未处理槽的绝对 dwt 时间,每消耗一槽(收到/超时/自己发完)+= interval;
    发送时机 = TREK 算术式 `(remainingRespToRx + anc_id == MAX_AHCHOR_NUMBER-1) && lastTxFcode != ANCH_RESP`;
  - **`twrAnchor_sentHandle()`** 按 `lastTxFcode` switch 分流(ANCH_POLL/RESP2/FINAL → A2A,
    ANCH_RESP/default → 槽推进);
  - **不变式 LISTENER ⇔ remainingRespToRx == -1**:`anch_rxRenableImmdiate()` 统一复位
    `remaining/wait4final/lastTxFcode`,杂散事件不得开窗;
  - `rate_timing()` helper 临时收拢速率阶梯(数值不变,Phase B 整体替换);
- **`freertos.c`**:task_twrRun 定义/创建/栈水位删除;task_uwb 栈 1280→2048(净 RAM 不增)。

### 两处有意的行为变更(超出机械迁移)

1. `rxOkHandle` 的 `default` 分支原来空操作(收到未知功能码后接收机静默停收),
   现改为 `anch_rxRenableImmdiate()` 重新开窗(也是 Phase C 杂帧路径的前提);
2. final 超时判定从 `remaining == 0` 改为 `wait4final != 0`,杂散超时不再误开 final 窗口。

时序数值完全未变:`nextSlotTime` 展开后与原 `FIRST_RESP_SEND_* + n*interval` 公式逐项等价。

### 真机回归清单(用户执行,未完成项随 Phase B 一起验)

- [ ] 双基站 + 标签:T2A 成功率、距离稳定性与改前一致,报警门限/语音/CAN 正常;
- [ ] 三基站:A2A 距离正常,A0 周期发起不间断;
- [ ] `task_rtosMonitor` 确认 task_uwb 栈水位(接管状态机后负载变重);
- [ ] 长跑 30min 无"单基站停收发"复发(`dw_main.c` ARFE/双 buffer 根因注释)。

## Phase B:`twr_set_replydelay()` 统一时序计算 🔶

**参照**:TREK §9(instance_set_replydelay)、§2.1(sfConfig_t)

1. `dw_instance.h` 新增 `twrTimings_t`(作为 `instance_data_t` 成员 `timings`):
   `firstRespDly_us / replyInterval_us / ancRespTxBack_us / finalTxBack_us /
   respRxTimeout_us / finalRxTimeout_us / pollRx2FinalRx_dwt / rngInitTxDly_us(Phase C 用)`;
2. `dw_main.c` 实现 `twr_set_replydelay(inst)`(init 在 dwt_configure 后调一次):
   - **`TWR_TIMING_LEGACY=1`(默认)**:从现有旧宏装填,空口时序与现网 Tag 完全一致(回归用);
     旧宏(`FIRST_RESP_SEND_*` 等)保留,唯一引用点就是这里的装填代码;
   - **`=0`(公式模式,Tag 端适配后启用)**:Trek 公式适配 N 基站:
     `preamble_us = (plen_symbols + sfdlen) × 1.01763(PRF64)`(sfdlen 按 dataRate:850K=16/6M8=8/110K=64),
     `frame_us(len) = preamble_us + calc_length_data(len)/1000`(**复用现成 calc_length_data,首次投入使用**),
     `replyInterval_us = frame_us(RESP_MSG_LEN) + TWR_TURNAROUND_US`,
     `respRxTimeout_us / finalRxTimeout_us = frame_us(对应帧长) + margin`;
     `TWR_TURNAROUND_US` 唯一可调余量,初值 500,按 SWO 实测(poll RX 回调进入→starttx 返回)收紧;
   - boot log 打印两种模式的时序对照表(LEGACY=1 时装填值 == 旧常量);
3. 替换消费点:删 `rate_timing()` helper 与 A2A 路径全部旧宏/全局引用;
   全局 `inst_resp_rx_timeout/inst_final_rx_timeout/inst_data_interval/inst_poll2final_time`
   收编进 `inst->timings`(`inst_one_slot_time` 保留,超帧输入);
   init 校验"单次 TWR 总时长 < slotDuration",超出 log_e;
4. 新文档 `docs/TWR_TIMING.md`:完整公式、参数表、双端必须一致的量(Tag 工程适配依据);
5. **验收**:LEGACY=1 编译烧录,T2A + A2A 与 Phase A 表现一致(时序值逐项相同);
   `TWR_TIMING_LEGACY=0` 留待 Tag 端按 `docs/TWR_TIMING.md` 适配后双端联调。

## Phase C:Discovery 基站侧(Trek 原版 ISO 0xC5)⬜

**参照**:TREK anchor discovery(instance_anch.c)

1. **帧过滤(仅 A0/gateway 放开)**:DW1000 `dwt_enableframefilter(... | DWT_FF_RSVD_EN)`(0x040),
   DW3000 `dwt_configureframefilter(..., | DWT_FF_RSVD_EN)`(0x010);A1/A2 不动;
   DW3000 必须实测:非法 reserved 帧要走 rxfailedcallback 且接收机被正确重开(此前有 ARFE 静默停收教训);
2. **blink 识别**:`rxOkHandle` 的 `switch(f_code)` **之前**特判 `rx_buffer[0]==0xC5 && frame_len==12`
   (blink 无 fcode/PAN),仅 `gatewayAnchor && twr_mode==LISTENER` 时处理,否则 re-enable;
3. **tagList 管理**:`anch_add_tag_to_list(uint8 *eui64)` 移植 Trek 原逻辑(顺序扫描,slot 即分配的
   tag_id/时隙号,上限 `MAX_TAG_NUMBER`,RAM 驻留,A0 重启后标签重新 blink 注册);
4. **RNG_INIT 发送**:帧控 `0x41 0x8C`(64bit 目的=标签 EUI + 16bit 源);载荷 = fcode 0x38 +
   sleepCorrection(int16) + tag_id(2B);sleep correction 从 `anch_perpareAnc2TagResp()` 提取公共函数
   `calc_tag_sleep_correction(slot, nowMs)`,RNG_INIT 路径额外 `+ sfPeriod`(Trek:最小 1.5 周期);
   `twr_mode=RESPONDER_B`,delayed TX @ blinkRxTs + `timings.rngInitTxDly_us`;sentHandle 中
   RESPONDER_B → 回 LISTENER + re-enable;
5. **验证**:Tag 端未就绪前用另一台设备发构造 blink 帧,A0 注册 tagList 并发 RNG_INIT(SWO log +
   第三台监听抓帧);A1/A2 对 blink 无反应。

## Phase D:冗余清理 ⬜

1. 删除未使用的 `instStatus` 枚举(STA_* 全部无引用;保持事件驱动 + twrMode 模型,不引入 TA_* 大状态机);
2. `availableAlgorithms[]` 两条目同指一个算法 + dummy:精简为单算法直接绑定(保留 chipType 字段);
3. 拼写修正(限触碰过的行):`FIANL_MSG_LEN`→`FINAL_MSG_LEN`、`anch_perpareAnc2TagResp`→
   `anch_prepareAnc2TagResp`、`anch_rxRenableImmdiate`→`anch_rxReenableImmediate`;
4. `dw_instance.h` 对外 extern 区最终审视;更新 `CLAUDE.md` 任务表与本文档;
5. **回归验证**(硬件):三级报警门限/语音/CAN;三基站 A2A 误差 < ±30cm(天线延迟已校准前提,
   见 `docs/antenna_delay_calibration.md`);长跑 30min;栈水位复核。

---

## 风险与约束

1. **空口兼容**:`TWR_TIMING_LEGACY=1` 期间与现网 Tag 完全兼容;切 0 必须与 Tag 工程同步刷机;
2. **task_uwb 单任务化**:handler 全程在 ISR 优先级任务内运行,耗时必须 < 最小时间窗;
   `TWR_TURNAROUND_US` 是守护余量,Phase B 用 SWO 时戳实测后定值;
3. **帧过滤放开 reserved 仅限 A0**,DW3000 上必须实测(ARFE 教训);
4. **回调内 starttx 失败路径**(delayed TX 过点)沿用 `rnganch_change_back_to_anchor`/
   `anch_rxRenableImmdiate` 兜底,不新增行为。
