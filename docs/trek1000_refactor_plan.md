# TREK1000 参考重构方案(4 阶段)

> **目的**:参照 DecaWave TREK1000 官方实现(规格见 `.claude/TREK1000_TWR_STATE_MACHINE_REFERENCE.md`),
> 重构本项目 UWB TWR 基站代码,统一实例管理、事件流、时序计算,消除多年累积的全局变量散落与重复代码。
>
> **本文档是跨电脑工作交接的唯一事实来源**。每完成一个阶段,更新本文档的进度标记并随代码一起提交。
>
> ⚠️ **说明**:Phase A/B 的内容还原自提交记录(`4542790`、`f816688`);Phase C/D 是根据代码内
> 遗留注释和 TREK1000 参考反推重拟的。如与你原始方案有出入,以下次核对后更新的版本为准。

---

## 进度总览(2026-07-06 更新)

| 阶段 | 内容 | 状态 | 提交 |
|------|------|------|------|
| Phase A | `instance_data_t` 统一 + 事件流重构 | ✅ 已提交(接口层) | `4542790` |
| Phase B | 回调直驱状态机 + 移除事件队列 | ⚠️ **半途,当前编译不过** | `f816688` |
| Phase C | `twr_set_replydelay()` 统一时序计算 | ⬜ 未开始 | — |
| Phase D | TDMA/A2A 调度完善 + 清理验证 | ⬜ 未开始 | — |

### ❗当前断点现场(下次开工第一件事)

`f816688` 提交时 Phase B 只完成了**接口签名迁移**,函数体迁移中断,当前 `cmake --build build` 失败:

1. **`dw_instance_anchor.c` 函数体未迁移完**(约 7 处函数):
   - `twrAnchor_rxOkHandle`(L131 起)、`anch_txRespOrRxReEnable`(L626 起)、
     `anch_rxRenableImmdiate`(L736)、`rnganch_change_back_to_anchor`(L814)、
     `anch_start_a2a`(L825)等定义仍是 `(void)` 签名或 `dwDevice_t *dev` 参数,
     与文件头部已改好的 `instance_data_t *inst` 原型冲突;
   - 函数体内仍引用已删除符号:`get_the_local_structure_of_dev()`、`dwDevice_t`、
     全局 `poll_rx_ts / resp_tx_ts / final_rx_ts / range_nb / recv_tag_id / handleResp_times / resp_valid`
     → 需改为 `inst->` 对应字段(字段映射见 Phase A 一节);
   - 旧字段 `dev->respTxIndex / dev->rxOtherResp / dev->indexDiff_*` 在新结构中已删,
     迁移时按 Phase B 设计改用 `inst->remainingRespToRx + rxRespMask` 轮转逻辑(TREK §4.2)。
2. **`freertos.c` L156-165、L277 附近**:仍在定义/创建已删除的 `task_twrRun`
   → 删除线程定义、创建调用及其静态栈(Phase B 设计:回调内直驱状态机,该任务已无存在意义)。
3. 完成后 `cmake --build build` 须零错误,DW1000/DW3000 两个目标都要过。

---

## 背景:重构前的问题

- TWR 轮转状态散落在 `dwDevice_t` + 十几个文件级全局变量(`frame_seq_nb`、`range_nb`、
  `poll_rx_ts`、`handleResp_times`、A2A 一整套 `a2a_*` 静态变量)中,T2A 与 A2A 各维护一份;
- 事件流走 `ISR → sema_uwbInt → task_uwb → queue_uwbEvent → task_twrRun → onEvent`,
  比 TREK1000 多一次队列转发和上下文切换,挤占延迟发送时间窗;
- 速率相关时序参数(`FIRST_RESP_SEND_*` 等 9 个宏)在 T2A/A2A 路径重复展开 4+ 处 if-else 阶梯,
  TREK1000 是 `instance_set_replydelay()` 一个函数按帧长公式统一算出。

---

## Phase A:`instance_data_t` 统一 + 事件流重构 ✅

**参照**:TREK §1(instance_data_t)、§8.3(事件队列语义)

- `dwDevice_t` → `instance_data_t`(单例,`instance_get()` 访问),收编:
  - 角色管理:`device_mode / twr_mode / device_id / gatewayAnchor`
  - 轮转状态:`remainingRespToRx / rxRespMask / wait4final / lastTxFcode / frame_seq_nb /
    range_nb / a2a_range_nb / recv_tag_id / resp_valid / nextSlotTime`
  - 时间戳:responder 侧 `poll_rx_ts / resp_tx_ts / final_rx_ts`,
    initiator 侧 `poll_tx_ts / resp_rx_ts[] / final_tx_time / final_rx_time`
  - `prev_range[]`、A2A 的 `a2a_distance[] / sframePeriod_ms / a2aStartTime_ms`
- 设计决策:**T2A responder / A2A initiator / A2A responder 共用同一套交换状态**
  (同一时刻只处于一个交换中,由 `device_mode + twr_mode` 区分语义,TREK1000 同款);
- 对外上报的全局变量(`anc_id / distance_report / range_status` 等)保留,供
  `net_protocol.c / app_network.c / dw_sort.c` 消费;
- `uwbAlgorithm_t` 回调签名改为 `instance_data_t *inst`。

## Phase B:回调直驱状态机 + 移除事件队列 ⚠️(收尾见"断点现场")

**参照**:TREK §8(callbacks 内直接准备 resp、调度 delayed TX/RX)

- 删除 `queue_uwbEvent` 与 `task_twrRun`,事件流简化为:
  `DW IRQ → sema_uwbInt → task_uwb → process_deca_irq → dwt_isr → callbacks 直调 onEvent`;
- A2A 周期触发改由 `task_uwb` 的 `osSemaphoreAcquire(sema_uwbInt, sframePeriod_ms)`
  超时兜底调用 `anch_checkA2ATrigger()`(仅 A0 需要,其余基站 `osWaitForever` 纯中断驱动);
- `rate_timing()` helper 合并 T2A 路径的速率 if-else 阶梯(A2A 路径的 3 处重复留给 Phase C);
- **剩余工作** = 断点现场清单 1~3。

## Phase C:`twr_set_replydelay()` 统一时序计算 ⬜

**参照**:TREK §9(instance_set_replydelay)、§2.1(sfConfig_t)

**目标**:所有 TX/RX 时序由帧长公式推导,消除 magic number 宏阶梯。

1. 新建 `twr_set_replydelay()`(建议放 `dw_main.c`,init 时按当前 `inst_dataRate` 调一次):
   - 用现成的 `calc_length_data()`(帧空中时长,ns)+ `RX_RESPONSE_TURNAROUND` 推导:
     - `fixedReplyDelay`(resp 槽间距)= preamble + resp 帧时长 + turnaround(TREK §9.2)
     - 首 resp 延时、anchor 回发延后量、final 延后量、
       `tagRespRxDelay / ancRespRxDelay`(RX 开启延时)
     - 帧等待超时:`fwto4RespFrame / fwto4FinalFrame`
   - 计算结果存入 `instance_data_t` 新增的时序字段(或独立 `twrTiming_t` 成员);
2. 替换消费点:
   - `rate_timing()` helper 及 A2A 路径残留的 3 处 `FIRST_RESP_SEND_*` if-else 阶梯
     (`dw_instance_anchor.c` L305/L469/L539 附近)全部改读统一时序字段;
   - 全局 `inst_resp_rx_timeout / inst_final_rx_timeout / inst_poll2final_time` 收编进时序结构;
   - `sfConfig.pollTxToFinalTxDly_us` 由公式生成,不再手写 `1300 + 3*1600`;
3. 删除 `dw_instance.h` 中 `FIRST_RESP_SEND_* / ANC_RESP_SEND_BACK_* / TAG_FINALE_SEND_BACK_*` 9 个宏;
4. **验收**:算出的各时序值与原宏值打印比对(允许微小偏差但须解释),实测 T2A + A2A 测距成功率不劣化。

## Phase D:TDMA/A2A 调度完善 + 清理验证 ⬜

**参照**:TREK §2(超帧)、§3.4(sleep correction)、§5(A2A 调度)

1. **时隙同步下发**:A0 在 resp 帧中携带 sleep correction(TREK §3.4 公式,
   `currentSlotTime = tick % sfPeriod`,`expected = tagAddr * slotDuration`),
   为 Tag 端 TDMA 收敛做好基站侧准备(resp 帧留好 `RES_TAG_SLP0/1` 字段位);
2. **A2A 调度对齐 TREK §5.3**:A1 收到 A0 final 后启动 `a1SlotTime` 定时器再发起 A1→A2
   (当前实现如与此不符则修正),A0 旁听 A2 resp 收齐 3 条 ToF;
3. **清理**:
   - `dw_instance.h` 对外 extern 区最终审视,能收编进 `instance_data_t` 的全部收编;
   - 更新 `CLAUDE.md` 任务表(删除 `task_twrRun` 行、`queue_uwbEvent` 条目,
     修正 task_uwb 职责描述)——**目前 CLAUDE.md 已过时,勿照它理解现状**;
4. **回归验证**(硬件):
   - 双基站 + 标签:T2A 三级报警距离门限触发正常,语音/CAN 输出正常;
   - A0/A1/A2 三基站:A2A 距离与卷尺实测误差 < ±30cm(天线延迟已按
     `docs/antenna_delay_calibration.md` 校准的前提下);
   - 长跑 30min 无"单基站停收发"复发(参考 `dw_main.c` L475 双 buffer 根因注释);
   - `task_rtosMonitor` 栈水位确认 `task_uwb` 扩栈需求(接管状态机后负载变重)。

---

## 提交约定

- 每阶段独立提交,格式:`参考Trek1000代码进行重构,session phase X: <内容> [YYYY-MM-DD HH:MM]`;
- 阶段内如需中断,提交信息注明"进行到一半"+ 断点位置,并**同步更新本文档的断点现场一节**;
- 编译不过的中间态尽量不留在 develop 最新提交(本次 Phase B 是意外,Claude 服务中断所致)。
