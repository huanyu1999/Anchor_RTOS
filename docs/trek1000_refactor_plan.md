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

## 进度总览(2026-07-12 更新)

| 阶段 | 内容 | 状态 | 提交 |
|------|------|------|------|
| Phase A | `instance_data_t` 统一 + 回调直驱事件流 | ✅ 已提交(待真机回归) | `4542790`+`f816688`(中间态)+`1fe8e48`(收尾) |
| Phase B | `twr_set_replydelay()` 统一时序计算(TREK 单轨返工) | ✅ 已提交 | `9b51b55`+`653ea38`(单轨返工) |
| Phase C | Discovery 基站侧(ISO 0xC5 blink) | 🔶 代码完成,待编译+烧录验证 | 工作区 |
| Phase D | 冗余清理 | ⬜ 未开始 | — |

### 当前断点

无代码断点。Phase C 代码于 2026-07-12 全部完成(工作区未提交),等待用户编译+烧录验证
(验证方法见 Phase C 第 5 条;Tag 端未就绪,可用另一台设备发构造 blink 帧)。通过后提交 → Phase D。

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
| 空口时序变更 | 允许。Tag 工程(`../Tag`,裸机)同步梳理更新;~~`TWR_TIMING_LEGACY` 开关做过渡~~ 2026-07-09 改判:双轨太乱,彻底删除旧宏,单轨 TREK 公式,接受 Tag 适配前 T2A 不可用的过渡期 |
| Blink 帧格式 | Trek 原版 ISO 0xC5 EUI-64 blink(12 字节);RNG_INIT 用长目的+短源混合帧头 |
| 延迟架构 | 时间关键路径移入回调(task_uwb 上下文),Trek 同款;task_twrRun 删除 |
| instance_data_t 范围 | 只收内部状态;对外全局保留:`anc_id/group_id/tag_id/distance_report[]/group_report[]/range_status/range_time/rx_power/ant_dly/distance_offset_cm/inst_ch/inst_prf/inst_dataRate` |
| 交换状态 | 时间戳共享(同一时刻只在一个交换中);TX-done 用 `lastTxFcode` 分流。2026-07-10 改判:**T2A/A2A 处理彻底分开**——处理函数按 TREK 三函数拆分,逐交换计数器/掩码按模式拆为 `remainingRespToRx`(T2A,LISTENER 不变式)+`remainingRespToRxAnc`/`rxRespMaskAnc`(A2A;TREK 掩码本就分开,计数器 TREK 共用、本工程为清晰起见拆开)(见 Phase B 第 5 条) |

---

## Phase A:`instance_data_t` 统一 + 回调直驱事件流 ✅

**参照**:TREK §1(instance_data_t)、§8(callbacks 内直驱状态机)

### 完成内容

- **`dw_instance.h`**:`dwDevice_t` → `instance_data_t`(单例,`instance_get()`),收编:
  角色(`device_mode/twr_mode/device_id/gatewayAnchor`)、
  轮转(`remainingRespToRx/rxRespMaskAnc/wait4final/lastTxFcode/frame_seq_nb/range_nb/
  a2a_range_nb/recv_tag_id/resp_valid/nextSlotTime`)、
  时间戳(responder `poll_rx_ts/resp_tx_ts/final_rx_ts`,initiator `poll_tx_ts/resp_rx_ts[]/
  final_tx_time/final_rx_time`)、`prev_range[]`、ANCRANGE 的 `a2a_distance[]/sframePeriod_ms/a2aStartTime_ms`;
- **`dw_main.c`**:4 回调直接调 `current_Algorithm->onEvent()`(task_uwb 上下文);
  `task_uwb` 循环 = `osSemaphoreAcquire(sema_uwbInt, A0?sframePeriod:forever)` + 尾部
  `anch_checkA2ATrigger()`;`queue_uwbEvent`/`task_twrRun` 删除(消除双任务并发访问 DW SPI 风险);
  `calc_length_data()` 110K 分支加 `USE_DW1000` 保护(修 DW3000 目标编译错);
- **`dw_instance_anchor.c`**:全函数 `inst->` 化,`a2aState_t`/`a2a_*` 静态副本/
  `handleResp_times`/`respTxIndex` 移除;核心机制:
  - **`anch_respSlotProcess()`**(替代 `anch_txRespOrRxReEnable`;2026-07-10 改名
    `anch_txresponse_or_rx_reenable`,TREK 同名)统一 resp 槽推进引擎:
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

## Phase B:`twr_set_replydelay()` 统一时序计算 🔶(TREK 单轨返工完成,待验证)

**参照**:TREK §9(instance_set_replydelay)、§2.1(sfConfig_t);
原版源码 `Reference Example\Trek1000-RTLS-master\trek1000\src\application\instance_common.c:848`

### 方案变更(2026-07-09)

首版 Phase B 采用"LEGACY 旧宏装填 + 公式计算"双轨并存、`TWR_TIMING_LEGACY` 开关切换,
新旧宏交叉过于混乱。经用户确认**推翻双轨方案**:彻底删除 `TWR_TIMING_LEGACY`、
`TWR_TURNAROUND_US` 与全部 18 个旧速率宏,`twr_set_replydelay()` 直接移植 TREK 原版
`instance_set_replydelay()`,单轨公式计算。**空口时序即刻改变,与现网 Tag 不兼容**
(Tag 工程按新版 `docs/TWR_TIMING.md` 适配后联调);A2A 双方同固件不受影响。

### 完成内容(2026-07-09)

1. **`dw_instance.h`**:删除 `TWR_TIMING_LEGACY/TWR_TURNAROUND_US` 与 18 个旧速率宏;
   新增 TREK 常量 `DW_RX_ON_DELAY(16us)`、`RX_RESPONSE_TURNAROUND(500us,TREK 原值 300,
   RTOS 保守起步,SWO 实测后收紧)`;`twrTimings_t` 整体替换为 TREK 字段集:
   `fixedReplyDelayAnc32h / preambleDuration32h / pollTx2FinalTxDelay32h /
   fixedReplyDelay_sy / fwto4RespFrame_sy / fwto4FinalFrame_sy`;
   调度字段改 32h 单位:`nextSlotTime32h / final_tx_time32h / final_rx_time32h`(uint32);
2. **`dw_main.c`**:`twr_set_replydelay()` 重写为 TREK 移植(帧长含 FCS_LEN,修正首版漏加;
   新增 `conv_us_to_devtime()` helper);T2A final 时刻 = `(N+1)×fixedReplyDelay` 公式导出
   (沿用 sfConfig 旧值 6100us 会与末槽 resp 空口重叠,核对时发现并改为公式;
   sfConfig 仍无消费者,Phase D 议题保留);boot log 单组输出 + 交换超 slot 的 log_e 检查;
3. **`dw_instance_anchor.c`**:全部时序消费点改 TREK 字段/32h 时间基:统一槽间隔
   (首槽=pollRx+1×,无 ancBack/finalBack 偏移)、delayed RX 开窗提前 `preambleDuration32h`、
   接收超时改 symbol 单位(顺带修正传 us 的 2.5% 偏差);A2A 槽位布局保留
   (首 responder 槽后移一槽、final 槽位 = N+1),Phase A 状态机结构与 A2A 三项容错修复不动;
4. **`docs/TWR_TIMING.md`** 重写:TREK 模型/字段表/公式/参考数值/Tag 适配清单;
5. **T2A/A2A 处理彻底分开(2026-07-10,用户提出 rxRespMask/remainingRespToRx 共用太乱)**:
   resp 收发处理按 TREK 三函数重组——T2A `anch_txresponse_or_rx_reenable()`(原
   `anch_respSlotProcess` 改 TREK 同名,槽推进 `nextSlotTime32h += fixed` 移入函数内,
   TREK 在函数内累加 delayedTRXTime32h 同款)、A2A 应答端 `anch_perpareAnc2AncResp()` +
   `rnganch_txResponseOrRxReenable()`(从 ANCH_POLL case 提取)、A2A 发起端
   `rnganch_rxRespSendfinal_or_rxReenable()`(resp2 接收与槽超时两路共用的
   "发 final/继续开窗/回 ANCHOR"三选一);逐交换状态按模式拆分:`rxRespMask` → A2A 专用
   `rxRespMaskAnc`(TREK rxResponseMaskAnc 同款),T2A 路径的两处掩码写入系死代码删除;
   计数器拆为 `remainingRespToRx`(T2A 专用,LISTENER ⇔ -1 不变式,A2A 期间保持 -1)+
   `remainingRespToRxAnc`(A2A 专用,`rnganch_changeBackToAnchor` 复位;TREK 共用一个,
   本工程按用户要求拆开);A2A 函数统一 `rnganch_` 前缀
   (`rnganch_start_a2a`/`rnganch_sendFinal`/`rnganch_changeBackToAnchor`)。
   行为等价迁移,唯一增强:发起端收 resp2 后重开接收失败时改为立即发 final 收尾
   (原裸调 rxenable 忽略返回值,失败会停摆到 A2A 兜底超时)。

### 历史(首版,已被返工取代)

2026-07-07 双轨版完成过:twrTimings_t(us 字段)+LEGACY 装填+公式对照打印;
A2A 容错修复三项(初始窗全覆盖/超时立即重开/收到末台直接发 final)——**此三项在返工版中保留**。

### 验收(用户执行)

- [ ] DW1000(及 DW3000)编译零错误,烧录;
- [ ] boot log `TWR timing:` 数值与 `docs/TWR_TIMING.md` §4 参考值一致
      (850K/前导1024:replyDelay≈1801us),无 `exceeds slot` 告警;
- [ ] 三基站 A2A:距离正常,A0 周期发起不间断,关掉 A1 容错路径仍工作;
- [ ] T2A 预期测不了距(Tag 未适配),但确认收到旧时序 poll 后走超时路径正确回 LISTENER,
      长跑无停收;
- [ ] Tag 工程(../Tag)按新版 `docs/TWR_TIMING.md` 适配后回归 T2A,SWO 实测收紧
      `RX_RESPONSE_TURNAROUND`。

## Phase C:Discovery 基站侧(Trek 原版 ISO 0xC5)🔶(代码完成,待验证)

**参照**:TREK anchor discovery(instance_anch.c)

### 完成内容(2026-07-12)

1. **帧过滤(仅 A0/gateway 放开)**:统一收口 `anch_setFrameFilter()`(dw_instance_anchor.c),
   `twrAnchor_Init`/`anch_rxRenableImmdiate` 及 dw_main.c 两处 boot 初始化共用:
   DW1000 `DWT_FF_RSVD_EN`(0x040)、DW3000 `DWT_FF_RSVD_EN`(0x010),按 `inst->gatewayAnchor`
   条件叠加,A1/A2 不动;DW3000 必须实测:非法 reserved 帧要走 rxfailedcallback 且接收机被
   正确重开(此前有 ARFE 静默停收教训);
2. **blink 识别**:`rxOkHandle` 的 `switch(f_code)` 之前特判 `rx_buffer[0]==0xC5 &&
   frame_len==BLINK_MSG_LEN+FCS_LEN`(blink 无 fcode/PAN),仅 `gatewayAnchor &&
   twr_mode==LISTENER` 时进 `anch_processTagBlink()`,否则 re-enable;特判放在 A2A 两个
   容错 guard **之后**——A2A 交换中收到 blink 走容错重开窗不打断交换,同时挡住 blink 的
   EUI 字节碰巧等于 0x7B 被 RESP2 分支误读时间戳的角落;
3. **tagList 管理**:`anch_add_tag_to_list(inst, eui64)` 移植 Trek 原逻辑(顺序扫描,已注册
   返回原 slot,slot 即分配的 tag_id/时隙号,上限 `MAX_TAG_LIST_SIZE=50`,RAM 驻留
   `inst->tagList[50][8]`+400B,A0 重启后标签重新 blink 注册);
4. **RNG_INIT 发送**:帧控 `0x41 0x8C`(64bit 目的=标签 EUI + 16bit 源 0x8000|anc_id),
   `RNG_INIT_MSG_LEN=20`(旧 `INIT_MSG_LEN` 删除);载荷 = fcode 0x38 + sleepCorrection(int16)
   + tag_id(2B),**小端**(TREK RES_TAG_SLP0/ADD0 原版布局;注意与本工程 RESP 帧 sleepCorr
   大端不同,Tag 端按帧类型区分);sleep correction 从 `anch_perpareAnc2TagResp()` 提取公共
   函数 `calc_tag_sleep_correction(slot, nowMs)`,RNG_INIT 路径额外 +1 周期(Trek:最小 1.5
   周期);`twr_mode=RESPONDER_B`,delayed TX @ blinkRx32h + `fixedReplyDelayAnc32h`
   (方案原文写 `timings.rngInitTxDly_us`,该字段在 Phase B 单轨返工后不存在,TREK 原版
   本就用 fixedReplyDelayAnc,直接沿用,不新增字段);sentHandle 增 `RTLS_MSG_RNG_INIT`
   分支 → `anch_rxRenableImmdiate`(RESPONDER_B 回 LISTENER);starttx 过点失败同样回
   LISTENER,标签会再 blink;
5. **验证(用户执行)**:
   - [ ] DW1000(及 DW3000)编译零错误,烧录;
   - [ ] Tag 端未就绪前用另一台设备发构造 blink 帧(ISO 0xC5:fctrl 0xC5 + seq + EUI64,
         共 10B+FCS),A0 SWO 出 `DISC: A0 rxd blink ... -> slot N` 并发 RNG_INIT
         (第三台监听抓帧);同一 EUI 重复 blink 返回同一 slot;
   - [ ] A1/A2 对 blink 无反应(帧过滤拒绝,DW3000 确认走 rxfailedcallback 后正常重开);
   - [ ] 三基站 A2A 回归:blink 混入时 A2A 距离输出不间断;
   - [ ] T2A/A2A 长跑无停收。

## Phase D:冗余清理 ⬜

1. 删除未使用的 `instStatus` 枚举(STA_* 全部无引用;保持事件驱动 + twrMode 模型,不引入 TA_* 大状态机);
2. `availableAlgorithms[]` 两条目同指一个算法 + dummy:精简为单算法直接绑定(保留 chipType 字段);
   `sfConfig` 全局无任何消费者(Phase B 查证),删除或接入超帧逻辑二选一;
3. 拼写修正(限触碰过的行):`FIANL_MSG_LEN`→`FINAL_MSG_LEN`、`anch_perpareAnc2TagResp`→
   `anch_prepareAnc2TagResp`、`anch_rxRenableImmdiate`→`anch_rxReenableImmediate`;
4. `dw_instance.h` 对外 extern 区最终审视;更新 `CLAUDE.md` 任务表与本文档;
5. **回归验证**(硬件):三级报警门限/语音/CAN;三基站 A2A 误差 < ±30cm(天线延迟已校准前提,
   见 `docs/antenna_delay_calibration.md`);长跑 30min;栈水位复核。

---

## 风险与约束

1. **空口兼容**:2026-07-09 起旧时序已彻底删除,现网 Tag 必须按 `docs/TWR_TIMING.md`
   适配刷机后 T2A 才可用(过渡期已获用户确认);
2. **task_uwb 单任务化**:handler 全程在 ISR 优先级任务内运行,耗时必须 < 最小时间窗;
   `RX_RESPONSE_TURNAROUND`(=500us)是守护余量,SWO 时戳实测后收紧(TREK 裸机原值 300);
3. **帧过滤放开 reserved 仅限 A0**,DW3000 上必须实测(ARFE 教训);
4. **回调内 starttx 失败路径**(delayed TX 过点)沿用 `rnganch_changeBackToAnchor`/
   `anch_rxRenableImmdiate` 兜底,不新增行为。
