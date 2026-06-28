# TREK1000 TWR State Machine — Complete Porting Reference

> This document is a self-contained specification of the DecaWave TREK1000/EVB1000 TWR (Two-Way Ranging) state machine,
> extracted from the `trek1000/src/application/` source code. It covers all core subsystems: Tag-to-Anchor ranging,
> Anchor-to-Anchor ranging, Tag Discovery (Blink), TDMA superframe scheduling, and Deep Sleep low-power management.
> It is intended as input context for Claude Code when porting these features to a new DW1000/DW3000-based project.

---

## 1. Architecture Overview

### 1.1 Roles and Modes

```c
typedef enum instanceModes { TAG, ANCHOR, ANCHOR_RNG, NUM_MODES } INST_MODE;
```

| Mode | Description |
|------|-------------|
| `TAG` | Initiates TWR ranging exchanges with anchors. Sleeps between exchanges. |
| `ANCHOR` | Default anchor mode. Listens for polls, responds, receives finals. |
| `ANCHOR_RNG` | Temporary sub-mode of ANCHOR. Anchor acts as TWR initiator for anchor-to-anchor ranging. |

Role assignment at boot is determined by DIP switch bit `SWS1_ANC_MODE`:
```c
if ((s1switch & SWS1_ANC_MODE) == 0)
    instance_mode = TAG;
else
    instance_mode = ANCHOR;
```

### 1.2 TWR Sub-Roles (within a ranging exchange)

```c
typedef enum instanceTWRModes {
    INITIATOR,    // Sends Poll, collects Responses, sends Final
    RESPONDER_A,  // Anchor responding to another anchor's poll (anchor-to-anchor)
    RESPONDER_B,  // Gateway anchor responding to a tag's Blink (discovery)
    RESPONDER_T,  // Anchor responding to a tag's poll (normal tag ranging)
    LISTENER,     // Idle/default — not participating in any exchange
    GREETER       // Tag sending Blinks during discovery phase
} ATWR_MODE;
```

### 1.3 Application State Machine States

```c
typedef enum inst_states {
    TA_INIT,                      // 0 - One-time initialization
    TA_TXE_WAIT,                  // 1 - Decide: sleep or proceed to TX
    TA_TXBLINK_WAIT_SEND,         // 2 - Configure and send Blink (discovery)
    TA_TXPOLL_WAIT_SEND,          // 2 - Configure and send Poll
    TA_TXFINAL_WAIT_SEND,         // 3 - Configure and send Final
    TA_TXRESPONSE_WAIT_SEND,      // 4 - Placeholder (response sent from ISR callback)
    TA_TX_WAIT_CONF,              // 5 - Wait for TX done confirmation
    TA_RXE_WAIT,                  // 6 - Enable receiver
    TA_RX_WAIT_DATA,              // 7 - Wait for incoming frame
    TA_SLEEP_DONE,                // 8 - Sleeping, waiting for wakeup timer
    TA_TXRESPONSE_SENT_POLLRX,    // 9 - TX conf after response to tag poll
    TA_TXRESPONSE_SENT_RESPRX,    // 10 - TX conf after response to anchor response
    TA_TXRESPONSE_SENT_TORX,      // 11 - TX conf after timeout-triggered response
    TA_TXRESPONSE_SENT_APOLLRX,   // 12 - TX conf after response to anchor poll
    TA_TXRESPONSE_SENT_ARESPRX    // 13 - TX conf after response to anchor response (A2A)
} INST_STATES;
```

### 1.4 Message Function Codes

```c
#define RTLS_DEMO_MSG_RNG_INIT    0x71  // Ranging init (discovery response: anchor → tag)
#define RTLS_DEMO_MSG_TAG_POLL    0x81  // Tag poll (broadcast)
#define RTLS_DEMO_MSG_ANCH_RESP   0x70  // Anchor response to tag poll
#define RTLS_DEMO_MSG_TAG_FINAL   0x82  // Tag final (with timestamps)
#define RTLS_DEMO_MSG_ANCH_POLL   0x7A  // Anchor-to-anchor poll
#define RTLS_DEMO_MSG_ANCH_RESP2  0x7B  // Anchor response to anchor poll
#define RTLS_DEMO_MSG_ANCH_FINAL  0x7C  // Anchor final (anchor-to-anchor)
```

### 1.5 Compile-Time Feature Flags

```c
#define DEEP_SLEEP        1  // Enable DW1000 deep sleep between ranging exchanges
#define CORRECT_RANGE_BIAS 1 // Compensate for accumulator growth bias at close range
#define ANCTOANCTWR       1  // Enable anchor-to-anchor TWR in last 2 superframe slots
#define DISCOVERY         0  // Tag discovery via Blink/RangingInit (0=disabled, tags use fixed addresses)
#define TAG_HASTO_RANGETO_A0 0 // If 1, tag only sends Final if A0's response received
#define READ_EVENT_COUNTERS 0  // Debug: read DW1000 event counters periodically
```

### 1.6 Key Addressing

```c
#define GATEWAY_ANCHOR_ADDR  0x8000  // A0 (gateway anchor)
#define A1_ANCHOR_ADDR       0x8001
#define A2_ANCHOR_ADDR       0x8002
#define A3_ANCHOR_ADDR       0x8003
#define MAX_ANCHOR_LIST_SIZE 4
#define MAX_TAG_LIST_SIZE    8       // (100 when DISCOVERY==1)
```

---

## 2. TDMA Superframe Structure

### 2.1 Configuration

```c
typedef struct {
    uint16 slotDuration_ms;       // Time for 1 tag to range to 4 anchors
    uint16 numSlots;              // Total slots in superframe (e.g. 10: 8 tag + 2 A2A)
    uint16 sfPeriod_ms;           // Superframe period = numSlots × slotDuration
    uint16 tagPeriod_ms;          // Tag sleep+range cycle (== sfPeriod in TREK)
    uint16 pollTxToFinalTxDly_us; // Poll TX to Final TX delay
} sfConfig_t;
```

### 2.2 Four TREK Modes

| Mode | Channel | Data Rate | Preamble | Slot Duration | Num Slots | SF Period | Location Rate |
|------|---------|-----------|----------|---------------|-----------|-----------|---------------|
| 1 | 2 | 110 Kbps | 1024 | 28 ms | 10 | 280 ms | ~3.6 Hz |
| 2 | 2 | 6.81 Mbps | 128 | 10 ms | 10 | 100 ms | 10 Hz |
| 3 | 5 | 110 Kbps | 1024 | 28 ms | 10 | 280 ms | ~3.6 Hz |
| 4 | 5 | 6.81 Mbps | 128 | 10 ms | 10 | 100 ms | 10 Hz |

With `DISCOVERY==1`, Mode 2 changes to: numSlots=100, sfPeriod=1000ms, for 100 tag slots.

### 2.3 Superframe Layout

```
|<------ Superframe Period (sfPeriod_ms) ------>|
| Slot 0 | Slot 1 | ... | Slot 7 | Slot 8 | Slot 9 |
| Tag 0  | Tag 1  | ... | Tag 7  | A0→A1,A2| A1→A2 |
|<---- 8 tag ranging slots ----->|<- 2 A2A slots ->|
```

- Each tag is assigned a slot (by address or discovery).
- Last 2 slots (`a0SlotTime_ms = (numSlots-2) × slotDuration`) reserved for anchor-to-anchor ranging.
- Tag sleep time = sfPeriod + sleepCorrection (from gateway anchor).

---

## 3. Tag-to-Anchor TWR Flow (Normal Ranging)

### 3.1 Protocol Sequence

```
Tag                A0              A1              A2              A3
 |---TAG_POLL(broadcast)------------------------------------------>|
 |                  |               |               |               |
 |<--ANCH_RESP-----|               |               |               |  (after fixedReplyDelay × 1)
 |                  |<--ANCH_RESP---|               |               |  (after fixedReplyDelay × 2)
 |                  |               |<--ANCH_RESP---|               |  (after fixedReplyDelay × 3)
 |                  |               |               |<--ANCH_RESP---|  (after fixedReplyDelay × 4)
 |                  |               |               |               |
 |---TAG_FINAL(broadcast, timestamps)------------------------------>|
```

### 3.2 Tag State Machine (tag_app_run)

```
TA_INIT
  └──► [TAG mode]
       ├── DISCOVERY==1: → TA_TXBLINK_WAIT_SEND (start with blinks)
       └── DISCOVERY==0: → TA_TXE_WAIT (instToSleep=TRUE, nextState=TA_TXPOLL_WAIT_SEND)

TA_TXE_WAIT
  ├── [instToSleep && nextState==TXPOLL/TXBLINK]:
  │     rangeNum++
  │     calc ranges from tofArray → newRange
  │     DW1000 enters deep sleep (dwt_entersleep)
  │     → TA_SLEEP_DONE (returns INST_DONE_WAIT_FOR_NEXT_EVENT_TO)
  └── [else]: → nextState directly

TA_SLEEP_DONE
  ├── [no wakeup timeout yet]: wait (INST_DONE_WAIT_FOR_NEXT_EVENT)
  └── [DWT_SIG_RX_TIMEOUT = wakeup timer fired]:
        port_wakeup_dw1000_fast()
        restore antenna delays, TX power
        → inst->nextState (TA_TXPOLL_WAIT_SEND or TA_TXBLINK_WAIT_SEND)

TA_TXBLINK_WAIT_SEND (DISCOVERY only)
  twrMode = GREETER
  Send Blink frame (0xC5, EUI-64 source address)
  Enable delayed RX (wait for Ranging Init)
  instToSleep = 1
  → TA_RX_WAIT_DATA (previousState = TA_TXBLINK_WAIT_SEND)

TA_TXPOLL_WAIT_SEND
  twrMode = INITIATOR
  Build TAG_POLL message (broadcast, range number)
  Set remainingRespToRx = MAX_ANCHOR_LIST_SIZE (4)
  Configure delayed RX after TX (tagRespRxDelay_sy)
  Configure RX frame wait timeout (fwto4RespFrame_sy)
  TX immediate with RESPONSE_EXPECTED
  → TA_TX_WAIT_CONF (previousState = TA_TXPOLL_WAIT_SEND)

TA_TX_WAIT_CONF
  Wait for DWT_SIG_TX_DONE
  ├── [previousState == TXFINAL]: → TA_TXE_WAIT (nextState=TXPOLL, instToSleep=TRUE)
  └── [previousState == TXPOLL]:
        Record pollTxTime, calculate finalTxTime
        Write PTXT and FTXT fields into Final message buffer
        → TA_RX_WAIT_DATA (fall through)

TA_RX_WAIT_DATA
  ├── DWT_SIG_RX_OKAY:
  │   └── RTLS_DEMO_MSG_ANCH_RESP:
  │         Record response RX timestamp in Final message
  │         rxResponseMask |= (1 << anchorID)
  │         remainingRespToRx--
  │         ├── [more responses expected]: re-enable delayed RX → stay
  │         └── [all received OR anchor 3]: → TA_TXFINAL_WAIT_SEND
  │
  ├── RTLS_DEMO_MSG_RNG_INIT (DISCOVERY only):
  │     Extract short address, sleep correction
  │     Set instanceAddress16 from ranging init
  │     → TA_TXE_WAIT (nextState=TXPOLL, instToSleep=TRUE)
  │
  └── DWT_SIG_RX_TIMEOUT:
        tag_process_rx_timeout():
        ├── [GREETER]: instToSleep=TRUE → TA_TXE_WAIT (nextState=TXBLINK)
        ├── [no responses at all]: instToSleep=TRUE → TA_TXE_WAIT (nextState=TXPOLL)
        ├── [error sending final]: instToSleep=TRUE → TA_TXE_WAIT (nextState=TXPOLL)
        └── [has some responses]: → TA_TXE_WAIT (nextState=TXFINAL)

TA_TXFINAL_WAIT_SEND
  Build TAG_FINAL (range number, rxResponseMask, all timestamps)
  Send as delayed TX (calculated finalTxTime)
  instToSleep = TRUE
  ├── [delayed TX failed (late)]: → TA_TXE_WAIT (nextState=TXPOLL, instToSleep=TRUE)
  └── [success]: → TA_TX_WAIT_CONF (previousState=TXFINAL)
```

### 3.3 Tag Sleep & Wakeup (tag_run)

```
tag_run() {
    while (INST_NOT_DONE_YET) { tag_app_run(); }

    if (INST_DONE_WAIT_FOR_NEXT_EVENT_TO) {
        // Calculate next wakeup time:
        nextPeriod = tagSleepRnd_ms + tagSleepTime_ms + tagSleepCorrection_ms;
        // tagSleepRnd_ms: random jitter (= slotDuration, cleared after first A0 response)
        // tagSleepTime_ms: nominal sleep (= tagPeriod_ms after discovery, or BLINK_PERIOD during blink)
        // tagSleepCorrection_ms: signed correction from gateway anchor to align to assigned slot
        nextWakeUpTime_ms = nextPeriod;
        instanceTimerEn = 1;  // start software timer
    }

    // Software timer check (polled in main loop):
    if (instanceTimerEn && (portGetTickCnt() - instanceWakeTime_ms) > nextWakeUpTime_ms) {
        instanceTimerEn = 0;
        putevent(DWT_SIG_RX_TIMEOUT);  // triggers TA_SLEEP_DONE exit
    }
}
```

### 3.4 Sleep Correction (Slot Synchronization)

Gateway anchor (A0) calculates a sleep correction in each Response to keep the tag in its assigned TDMA slot:

```c
// In anch_prepare_anc2tag_response():
currentSlotTime = uTimeStamp % sframePeriod_ms;     // where are we in the superframe?
expectedSlotTime = tagAddress * slotDuration_ms;     // where should this tag's poll arrive?
error = expectedSlotTime - currentSlotTime;          // signed offset

if (error < -(sframePeriod_ms / 2))
    tagSleepCorrection = sframePeriod_ms + error;    // wrap: add full period
else
    tagSleepCorrection = error;

// Correction is sent in Response message bytes RES_TAG_SLP0/RES_TAG_SLP1 (int16, ms)
```

Tag applies this correction to its next sleep duration, converging to the correct slot over a few cycles.

---

## 4. Anchor State Machine (Tag-to-Anchor perspective)

### 4.1 Anchor State Flow

```
TA_INIT [ANCHOR mode]
  Configure address, PAN ID, frame filter
  gatewayAnchor = TRUE if address == 0x8000
  Disable frame filter type (accept all frame types via DWT_FF_NOTYPE_EN)
  Configure ranging init frame headers (for discovery)
  Calculate a2aStartTime_ms (first A2A ranging time, skip 5 superframes)
  → TA_RXE_WAIT

TA_RXE_WAIT
  Enable RX (immediate, no timeout)
  → TA_RX_WAIT_DATA (fall through if message pending)

TA_RX_WAIT_DATA
  ├── DWT_SIG_RX_OKAY:
  │   switch(fcode):
  │
  │   ├── RTLS_DEMO_MSG_TAG_POLL:
  │   │     [Only if mode==ANCHOR and srcAddr < MAX_TAG_LIST_SIZE]
  │   │     twrMode = RESPONDER_T
  │   │     Prepare response (with previous ToF, sleep correction)
  │   │     remainingRespToRx = NUM_EXPECTED_RESPONSES (3)
  │   │     delayedTRXTime = pollRxTimestamp
  │   │     Call anch_txresponse_or_rx_reenable():
  │   │       - If it's this anchor's turn to respond: delayed TX → TA_TX_WAIT_CONF
  │   │       - Else: delayed RX to catch other anchors' responses → stay in TA_RX_WAIT_DATA
  │   │
  │   ├── RTLS_DEMO_MSG_ANCH_RESP:
  │   │     [Only if twrMode==RESPONDER_T — we're participating in this tag's exchange]
  │   │     rxResps++, remainingRespToRx--
  │   │     Call anch_txresponse_or_rx_reenable() again for next slot
  │   │
  │   ├── RTLS_DEMO_MSG_TAG_FINAL:
  │   │     [Only if twrMode==RESPONDER_T and mode==ANCHOR]
  │   │     Calculate ToF: calc_tof(messageData, anchorRespTxTime, finalRxTime, pollRxTime)
  │   │     Store tof[tag_index]
  │   │     Calculate ranges, store results
  │   │     twrMode = LISTENER
  │   │     Re-enable RX (no timeout) → TA_RXE_WAIT
  │   │
  │   ├── RTLS_DEMO_MSG_ANCH_POLL: (anchor-to-anchor, see §5)
  │   ├── RTLS_DEMO_MSG_ANCH_RESP2: (anchor-to-anchor, see §5)
  │   └── RTLS_DEMO_MSG_ANCH_FINAL: (anchor-to-anchor, see §5)
  │
  ├── DWT_SIG_RX_BLINK (DISCOVERY, gateway only):
  │     anch_add_tag_to_list(tagID) → slot assignment
  │     twrMode = RESPONDER_B
  │     Send Ranging Init (delayed TX) with slot number and sleep correction
  │     → TA_TX_WAIT_CONF or error recovery
  │
  └── DWT_SIG_RX_TIMEOUT:
        ├── [RESPONDER_T, wait4final]: twrMode=LISTENER → TA_RXE_WAIT
        ├── [RESPONDER_T, waiting for responses]: try send response or re-enable RX
        ├── [RESPONDER_A, wait4final]: twrMode=LISTENER → TA_RXE_WAIT
        └── [LISTENER/default]: re-enable RX immediately
```

### 4.2 Anchor Response Scheduling (anch_txresponse_or_rx_reenable)

Each anchor responds at a fixed delay from the Poll RX time, staggered by anchor index:

```
delayedTRXTime += fixedReplyDelayAnc32h;  // incremented per received/expected response

if (remainingRespToRx + shortAdd_idx == NUM_EXPECTED_RESPONSES):
    // It's our turn to transmit
    sendResp = 1

if sendResp:
    delayed TX at delayedTRXTime with RESPONSE_EXPECTED (auto-enable RX for Final)
else:
    delayed RX at delayedTRXTime (wait for next anchor's response, or Final)
```

Anchor A0 responds first (1× fixedReplyDelay), A1 at 2×, A2 at 3×, A3 at 4×.

### 4.3 ToF Calculation (DS-TWR formula)

```c
// Symmetric Double-Sided TWR:
Ra = anchorRespRxTime - tagPollTxTime;     // Tag's round trip (poll → resp)
Db = anchorRespTxTime - tagPollRxTime;     // Anchor's processing delay
Rb = tagFinalRxTime   - anchorRespTxTime;  // Anchor's round trip (resp → final)
Da = tagFinalTxTime   - anchorRespRxTime;  // Tag's processing delay (resp → final)

tof = (Ra × Rb - Da × Db) / (Ra + Da + Rb + Db);
// In 40-bit DW1000 timestamp ticks (~15.65 ps per tick)
```

---

## 5. Anchor-to-Anchor TWR (ANCTOANCTWR == 1)

### 5.1 Purpose

Auto-positioning: anchors measure distances between themselves for MDS-based coordinate recovery.
Uses the last 2 slots of each superframe. Lower priority than tag ranging.

### 5.2 Ranging Topology

- **Slot N-2**: A0 (gateway) ranges to A1 and A2 (A0 sends ANCH_POLL, A1/A2 respond with ANCH_RESP2, A0 sends ANCH_FINAL)
- **Slot N-1**: A1 ranges to A2 (A1 sends ANCH_POLL, A2 responds with ANCH_RESP2, A1 sends ANCH_FINAL)
- **A3** does not participate in anchor-to-anchor ranging.

### 5.3 Trigger Logic (in anch_run)

```c
// Gateway anchor (A0): checked every main loop iteration
if (gatewayAnchor && portGetTickCnt() >= a2aStartTime_ms) {
    a2aStartTime_ms += sframePeriod_ms;  // schedule next
    if (mode == ANCHOR && twrMode == LISTENER) {  // not busy with a tag
        anch_change_to_rnganchor(inst);
        // dest = A1 (0x8001)
    }
}

// A1: triggered after receiving A0's Final
if (instanceAddress16 == A1_ANCHOR_ADDR && instanceTimerEn > 0) {
    if (portGetTickCnt() >= a1SlotTime_ms) {
        if (mode == ANCHOR && twrMode == LISTENER) {
            anch_change_to_rnganchor(inst);
            // dest = A2 (0x8002)
        }
        instanceTimerEn = 0;
    }
}
```

### 5.4 Mode Switching

**ANCHOR → ANCHOR_RNG** (`anch_change_to_rnganchor`):
```c
port_DisableEXT_IRQ();
inst->mode = ANCHOR_RNG;
inst->twrMode = INITIATOR;
dwt_forcetrxoff();           // disable radio
instance_clearevents();       // flush event queue
inst->testAppState = TA_TXPOLL_WAIT_SEND;
// A0: remainingRespToRx = 2 (expects A1, A2)
// A1: remainingRespToRx = 1 (expects A2)
port_EnableEXT_IRQ();
```

**ANCHOR_RNG → ANCHOR** (`rnganch_change_back_to_anchor`):
```c
inst->testAppState = TA_RXE_WAIT;
inst->mode = ANCHOR;
inst->twrMode = LISTENER;
dwt_setrxtimeout(0);         // no timeout
dwt_setrxaftertxdelay(0);
```

### 5.5 Anchor-to-Anchor Protocol Sequence

```
A0 (INITIATOR)        A1 (RESPONDER)       A2 (RESPONDER)
 |---ANCH_POLL(broadcast, nextAnc=A1)------>|
 |                     |                     |
 |<----ANCH_RESP2------|                     |  (fixedReplyDelay × 1)
 |                     |<----ANCH_RESP2------|  (fixedReplyDelay × 2)
 |                     |                     |
 |---ANCH_FINAL(timestamps)---------------->|

Then A1 initiates:
A1 (INITIATOR)        A2 (RESPONDER)
 |---ANCH_POLL(broadcast)-->|
 |<----ANCH_RESP2-----------|
 |---ANCH_FINAL(timestamps)->|
```

### 5.6 A0 Collects All Three ToFs

A0 is the gateway and collects:
1. **A0–A1 ToF**: from its own exchange
2. **A0–A2 ToF**: from its own exchange
3. **A1–A2 ToF**: A0 eavesdrops on A2's ANCH_RESP2 to A1's poll (rxResps==3 triggers `rxResponseMaskAnc |= 0x8`)

When rxResps == 3, A0 calculates all three ranges and reports via `instance_calc_ranges(&tofArrayAnc[...], TOF_REPORT_A2A)`.

### 5.7 A2A State Machine Flow

```
TA_TXPOLL_WAIT_SEND (ANCHOR_RNG mode)
  Build ANCH_POLL (rangeNumAnc, nextAnchor address for gateway)
  TX immediate with RESPONSE_EXPECTED
  rxResponseMaskAnc = 0
  → TA_TX_WAIT_CONF (previousState = TA_TXPOLL_WAIT_SEND)

TA_TX_WAIT_CONF
  [previousState == TXPOLL]:
    Calculate pollTx2FinalTxDelayAnc-based finalTxTime
    Write PTXT/FTXT into Final message buffer
    → TA_RXE_WAIT → TA_RX_WAIT_DATA

TA_RX_WAIT_DATA [ANCHOR_RNG mode]
  ├── ANCH_RESP2 received:
  │     rxResps++, remainingRespToRx--
  │     Record response RX timestamp
  │     rxResponseMaskAnc |= (1 << anchorID)
  │     rnganchrxresp_signalsendfinal_or_rx_reenable():
  │       ├── [remainingRespToRx == 0]: DWT_SIG_DW_IDLE → send final
  │       └── [more expected]: delayed RX for next response
  │     → TA_TXFINAL_WAIT_SEND or stay in RX
  │
  └── TIMEOUT:
        ├── [no responses]: rnganch_change_back_to_anchor()
        ├── [error sending final]: rnganch_change_back_to_anchor()
        └── [some responses]: → TA_TXFINAL_WAIT_SEND

TA_TXFINAL_WAIT_SEND (ANCHOR_RNG mode)
  Build ANCH_FINAL (rangeNumAnc, rxResponseMaskAnc, timestamps)
  Delayed TX
  ├── [late/failed]: rnganch_change_back_to_anchor()
  └── [success]: → TA_TX_WAIT_CONF (previousState = TXFINAL)

TA_TX_WAIT_CONF
  [previousState == TXFINAL]:
    rnganch_change_back_to_anchor()  // back to normal ANCHOR + LISTENER
```

### 5.8 RESPONDER Side (A1/A2 receiving ANCH_POLL)

In `rx_ok_cb_anch` ISR callback:
```c
case RTLS_DEMO_MSG_ANCH_POLL:
    // A0 and A3 ignore (gateway doesn't respond to its own poll; A3 not involved)
    if (gatewayAnchor || instanceAddress16 > A2_ANCHOR_ADDR) {
        anch_no_timeout_rx_reenable(); return;
    }
    twrMode = RESPONDER_A;
    // Prepare ANCH_RESP2, schedule delayed TX
    // A1: responds at pollRxTime + fixedReplyDelay
    // A2: responds at pollRxTime + 2 × fixedReplyDelay (when A0's poll)
    //     or pollRxTime + fixedReplyDelay (when A1's poll)
    // After TX: enable delayed RX for Final
```

In `anch_app_run` application level:
```c
case RTLS_DEMO_MSG_ANCH_FINAL:  // received Final from initiating anchor
    if (twrMode == RESPONDER_A && mode == ANCHOR) {
        Calculate ToF → tofAnc[srcAddr]
        twrMode = LISTENER
        re-enable RX
    }
    // A1 also sets instanceTimerEn=1 upon receiving A0's Final → triggers A1→A2 ranging
```

---

## 6. Tag Discovery (DISCOVERY == 1)

### 6.1 Overview

When `DISCOVERY` is enabled, tags don't have pre-assigned short addresses. They start by broadcasting
Blinks with their 64-bit EUI until the gateway anchor assigns them a slot and short address.

### 6.2 Tag Discovery Flow

```
Tag boots → TA_INIT:
    tagSleepTime_ms = BLINK_PERIOD (2000 ms)
    Copy EUI-64 into blink message
    → TA_TXBLINK_WAIT_SEND

TA_TXBLINK_WAIT_SEND:
    twrMode = GREETER
    Send Blink (frame control 0xC5, EUI-64 source)
    Enable delayed RX (wait for Ranging Init)
    instToSleep = 1
    → TA_RX_WAIT_DATA

TA_RX_WAIT_DATA:
    ├── RTLS_DEMO_MSG_RNG_INIT received (from gateway):
    │     Extract short address from message → instanceAddress16
    │     Extract sleep correction
    │     tagSleepTime_ms = tagPeriod_ms (normal period)
    │     → TA_TXE_WAIT (nextState=TXPOLL, instToSleep=TRUE)
    │     // Tag is now "discovered" and switches to normal TWR mode
    │
    └── TIMEOUT:
          instToSleep = TRUE
          → TA_TXE_WAIT (nextState=TXBLINK)
          // Retry blink after sleep
```

### 6.3 Gateway Anchor Discovery Flow

```
rx_ok_cb_anch:
    if (gatewayAnchor && fctrl[0]==0xC5 && twrMode==LISTENER):
        rxd_event = DWT_SIG_RX_BLINK
        slot = anch_add_tag_to_list(tagID)  // find or allocate slot (0 to MAX_TAG_LIST_SIZE-1)
        if (slot != -1):
            twrMode = RESPONDER_B
            Prepare Ranging Init message:
              - dest: tag's EUI-64
              - source: anchor's short address
              - messageData: sleep correction (int16) + tag short address (= slot number)
            Delayed TX
            → TX confirmation, then twrMode back to LISTENER (in tx_conf_cb)
```

### 6.4 Ranging Init Message Layout

```
Frame type: data (0x41), long dest / short source addressing
Size: RANGINGINIT_MSG_LEN (5) + FRAME_CRTL_AND_ADDRESS_LS (15) + CRC (2) = 22 bytes

messageData[0] = FCODE = RTLS_DEMO_MSG_RNG_INIT (0x71)
messageData[1] = RES_TAG_SLP0 (sleep correction LSB, int16 ms)
messageData[2] = RES_TAG_SLP1 (sleep correction MSB)
messageData[3] = RES_TAG_ADD0 (assigned short address / slot LSB)
messageData[4] = RES_TAG_ADD1 (assigned short address / slot MSB)
```

### 6.5 Sleep Correction for Discovery

```c
currentSlotTime = uTimeStamp % sframePeriod_ms;
expectedSlotTime = slot * slotDuration_ms;
error = expectedSlotTime - currentSlotTime;

if (error < -(sframePeriod_ms >> 1))
    tagSleepCorrection = sframePeriod_ms + error;
else
    tagSleepCorrection = error;

tagSleepCorrection += sframePeriod_ms;  // minimum sleep = 1.5 periods
```

---

## 7. Deep Sleep & Low Power (DEEP_SLEEP == 1)

### 7.1 Sleep Entry (Tag side)

In `TA_TXE_WAIT`, when `instToSleep==TRUE` and `nextState==TA_TXPOLL_WAIT_SEND`:
```c
rangeNum++;              // increment before sleep
instDone = INST_DONE_WAIT_FOR_NEXT_EVENT_TO;  // signal to tag_run() to start timer
testAppState = TA_SLEEP_DONE;

// Calculate and report any received ranges:
if (rxResponseMask != 0) {
    newRange = instance_calc_ranges(&tofArray[0], MAX_ANCHOR_LIST_SIZE, TOF_REPORT_T2A, &rxResponseMask);
}

// Enter deep sleep:
dwt_entersleep();  // DW1000 enters DEEP SLEEP (nanoamps current)
```

### 7.2 Sleep Configuration

```c
uint16 sleep_mode = DWT_PRESRV_SLEEP | DWT_CONFIG | DWT_TANDV;
if (configData.txPreambLength == DWT_PLEN_64)
    sleep_mode |= DWT_LOADOPSET;
dwt_configuresleep(sleep_mode, DWT_WAKE_WK | DWT_WAKE_CS | DWT_SLP_EN);
```

- `DWT_PRESRV_SLEEP`: Preserve sleep bit after wake.
- `DWT_CONFIG`: Upload config on wake.
- `DWT_TANDV`: Preserve TX antenna delay and VMEAS values.
- `DWT_WAKE_WK`: Wake on WAKEUP pin.
- `DWT_WAKE_CS`: Wake on SPI CS assertion.
- `DWT_SLP_EN`: Enable sleep.

### 7.3 Wakeup Sequence (TA_SLEEP_DONE)

```c
// Triggered by software timer via DWT_SIG_RX_TIMEOUT event
instToSleep = FALSE;
testAppState = nextState;     // → TA_TXPOLL_WAIT_SEND
instanceWakeTime_ms = portGetTickCnt();

// Hardware wakeup:
port_wakeup_dw1000_fast();    // Assert CS/WAKEUP pin, wait for DW1000 XTAL to stabilize

// Post-wakeup restoration (not preserved across deep sleep):
dwt_settxantennadelay(txAntennaDelay);  // TX delay not preserved (MP bug)
dwt_seteui(eui64);                       // EUI not preserved unless in NVM (DISCOVERY==0 only)
instance_set_antennadelays();            // Apply any pending changes
instance_set_txpower();                  // Apply any pending TX power changes
```

### 7.4 Sleep Timing (tag_run)

```c
// After INST_DONE_WAIT_FOR_NEXT_EVENT_TO:
nextPeriod = tagSleepRnd_ms          // random initial jitter (= slotDuration, 0 after first sync)
           + tagSleepTime_ms         // nominal period (= tagPeriod_ms = sfPeriod_ms)
           + tagSleepCorrection_ms;  // signed correction from gateway (-0.5 to +1.5 periods)

nextWakeUpTime_ms = (uint32)nextPeriod;
tagSleepCorrection_ms = 0;  // clear after use
instanceTimerEn = 1;         // start software poll timer

// Polled in tag_run():
if (instanceTimerEn && (portGetTickCnt() - instanceWakeTime_ms) > nextWakeUpTime_ms) {
    instanceTimerEn = 0;
    putevent(DWT_SIG_RX_TIMEOUT);  // synthetic event to wake state machine
}
```

---

## 8. ISR / Callback Architecture

### 8.1 Callback Registration

```c
if (inst_mode == ANCHOR)
    dwt_setcallbacks(tx_conf_cb, rx_ok_cb_anch, rx_to_cb_anch, rx_err_cb_anch);
else
    dwt_setcallbacks(tx_conf_cb, rx_ok_cb_tag,  rx_to_cb_tag,  rx_err_cb_tag);
```

### 8.2 Callback Responsibilities

**tx_conf_cb** (shared):
- Read TX timestamp
- If `twrMode == RESPONDER_B`: set `twrMode = LISTENER` (blink response sent, no further action)
- If `twrMode == GREETER`: ignore (blink sent, waiting for ranging init)
- Otherwise: queue `DWT_SIG_TX_DONE` event

**rx_ok_cb_tag**:
- Parse frame control → determine addressing mode
- For `RTLS_DEMO_MSG_ANCH_RESP`: record RX timestamp, update `rxResponseMask`, call `tag_rx_reenable()` for next response or signal idle
- For `RTLS_DEMO_MSG_RNG_INIT` (DISCOVERY): queue for application processing
- All other message types: treat as error/unknown

**rx_ok_cb_anch**:
- For `TAG_POLL`: prepare response, calculate response TX delay, call `anch_txresponse_or_rx_reenable()`
- For `ANCH_RESP`: if RESPONDER_T, increment rxResps, re-schedule TX or RX
- For `TAG_FINAL`: calculate ToF, return to LISTENER
- For `ANCH_POLL` (A2A): prepare ANCH_RESP2, schedule delayed TX
- For `ANCH_RESP2` (A2A): if ANCHOR_RNG, record timestamp; if ANCHOR, eavesdrop for A1-A2 ToF
- For `ANCH_FINAL` (A2A): calculate ToF, store, enable A1→A2 timer
- For `BLINK` (DISCOVERY): add tag, send ranging init

**rx_to_cb / rx_err_cb**:
- Tag: call `tag_handle_error_unknownframe()` → decrement `remainingRespToRx`, re-enable RX or signal timeout
- Anchor: call `anch_handle_error_unknownframe_timeout()` → handle per mode (ANCHOR_RNG vs ANCHOR, RESPONDER_T vs RESPONDER_A vs LISTENER)

### 8.3 Event Queue

```c
#define MAX_EVENT_NUMBER 4

// ISR callbacks write events via instance_putevent()
// Application loop reads via instance_peekevent() / instance_getevent()
// Circular buffer with dweventIdxIn / dweventIdxOut / dweventPeek
```

The event queue decouples ISR-time actions (which must be fast — prepare TX buffer, schedule delayed TX/RX)
from application-time actions (calculate ToF, update state machine, report ranges).

---

## 9. Timing Parameters & Delay Calculations

### 9.1 Key Timing Fields (instance_data_t)

| Field | Description |
|-------|-------------|
| `pollTx2FinalTxDelay` | Tag: poll TX → final TX delay (device time units, 40-bit) |
| `pollTx2FinalTxDelayAnc` | Anchor A2A: poll TX → final TX delay (shorter, only 2 responses) |
| `fixedReplyDelayAnc32h` | Fixed response delay per anchor slot (high 32 bits of 40-bit time) |
| `tagRespRxDelay_sy` | Tag: delay from poll TX to first RX enable (symbols) |
| `ancRespRxDelay_sy` | Anchor: delay from response TX to RX enable for next frame (symbols) |
| `fwto4RespFrame_sy` | Frame wait timeout for response frames (symbols) |
| `fwto4FinalFrame_sy` | Frame wait timeout for final frames (symbols) |
| `fwtoTime_sy` | Frame wait timeout for A2A response (symbols) |
| `preambleDuration32h` | Preamble duration in device time (for delayed RX adjustment) |
| `anc1RespTx2FinalRxDelay_sy` | A1: delay from own response TX to expected Final RX |
| `anc2RespTx2FinalRxDelay_sy` | A2: delay from own response TX to expected Final RX |

### 9.2 instance_set_replydelay() Summary

```c
// Called with pollTxToFinalTxDly_us from sfConfig
pollTx2FinalTxDelay = convert_usec_to_devtime(delayus);
pollTx2FinalTxDelayAnc = convert_usec_to_devtime((delayus/2) + RX_RESPONSE_TURNAROUND);

// fixedReplyDelay = preamble + response frame duration + RX_RESPONSE_TURNAROUND (300 us)
fixedReplyDelayAnc32h = convert_usec_to_devtime(respframe + RX_RESPONSE_TURNAROUND) >> 8;

// Tag RX delay = turnaround + (response frame - poll frame) duration
tagRespRxDelay_sy = RX_RESPONSE_TURNAROUND + respframe_sy - pollframe_sy;

// Anchor RX delay = turnaround - RX on delay (16 us)
ancRespRxDelay_sy = RX_RESPONSE_TURNAROUND - DW_RX_ON_DELAY;
```

---

## 10. Message Layouts

### 10.1 Tag Poll

```
| Frame Ctrl (2) | Seq (1) | PAN ID (2) | Dest 0xFFFF (2) | Src (2) | FCODE=0x81 (1) | RangeNum (1) | CRC (2) |
Total: TAG_POLL_MSG_LEN(2) + FRAME_CRTL_AND_ADDRESS_S(9) + CRC(2) = 13 bytes
```

### 10.2 Anchor Response (to Tag)

```
| Frame Ctrl (2) | Seq (1) | PAN ID (2) | Dest (2) | Src (2) |
| FCODE=0x70 (1) | SleepCorr (2) | PrevToF (4) | PrevRangeNum (1) | CRC (2) |
Total: ANCH_RESPONSE_MSG_LEN(8) + 9 + 2 = 19 bytes
```

### 10.3 Tag Final

```
| Frame Ctrl (2) | Seq (1) | PAN ID (2) | Dest 0xFFFF (2) | Src (2) |
| FCODE=0x82 (1) | RangeNum (1) |
| PollTxTime (5) |
| Resp0_RxTime (5) | Resp1_RxTime (5) | Resp2_RxTime (5) | Resp3_RxTime (5) |
| FinalTxTime (5) |
| ValidRespMask (1) |
| CRC (2) |
Total: TAG_FINAL_MSG_LEN(33) + 9 + 2 = 44 bytes
```

### 10.4 Anchor Poll (A2A)

```
| Frame Ctrl (2) | Seq (1) | PAN ID (2) | Dest 0xFFFF (2) | Src (2) |
| FCODE=0x7A (1) | RangeNum (1) | [NextAnchor (2)] | CRC (2) |
Total (gateway): ANCH_POLL_MSG_LEN(4) + 9 + 2 = 15 bytes
Total (non-gateway): ANCH_POLL_MSG_LEN_S(2) + 9 + 2 = 13 bytes
```

### 10.5 Anchor Response (A2A)

Same structure as Anchor Response to Tag, but FCODE=0x7B.

### 10.6 Anchor Final (A2A)

Same structure as Tag Final, but FCODE=0x7C.

### 10.7 Blink (Discovery)

```
| Frame Ctrl 0xC5 (1) | Seq (1) | TagID EUI-64 (8) | CRC (2) |
Total: 12 bytes (ISO/IEC 802.15.4 blink)
```

### 10.8 Ranging Init (Discovery)

```
| Frame Ctrl (2) | Seq (1) | PAN ID (2) | Dest EUI-64 (8) | Src Short (2) |
| FCODE=0x71 (1) | SleepCorr (2) | TagShortAddr (2) | CRC (2) |
Total: RANGINGINIT_MSG_LEN(5) + FRAME_CRTL_AND_ADDRESS_LS(15) + CRC(2) = 22 bytes
```

---

## 11. Porting Checklist

When porting to a new project (different MCU, different UWB chip like DW3000, or different topology):

### 11.1 Must Implement
- [ ] State machine enum definitions (INST_STATES, INST_MODE, ATWR_MODE)
- [ ] Event queue (ISR→application decoupling, circular buffer)
- [ ] Tag state machine: INIT → POLL → wait responses → FINAL → SLEEP → repeat
- [ ] Anchor state machine: INIT → RX → respond to poll → wait for final → calculate ToF
- [ ] Delayed TX/RX scheduling (critical for deterministic response timing)
- [ ] DS-TWR ToF calculation formula
- [ ] Sleep timer (software timer polling or hardware RTC/alarm)

### 11.2 Should Implement
- [ ] TDMA superframe with configurable slot duration and count
- [ ] Sleep correction from gateway anchor for slot synchronization
- [ ] Deep sleep entry/exit with DW config preservation
- [ ] Antenna delay calibration (OTP or compile-time)
- [ ] Range bias correction

### 11.3 Optional Features
- [ ] Tag Discovery (Blink → Ranging Init → assigned address)
- [ ] Anchor-to-Anchor ranging (auto-positioning)
- [ ] Multiple operating modes (channel/data rate/preamble configurations)
- [ ] USB CDC range reporting

### 11.4 Key Adaptations Needed
- **DW3000**: Different register set, different API names (`dwt_` → `dw3000_` or similar). Timestamp format may differ. STS/SFD options are different.
- **Different MCU**: Replace `port.c/.h` (SPI driver, GPIO, IRQ, sleep, tick timer). Replace `deca_spi.c`.
- **Different topology**: Adjust `MAX_ANCHOR_LIST_SIZE`, `NUM_EXPECTED_RESPONSES`, response delay scheduling. For >4 anchors, extend the Final message timestamp array.
- **Different RTOS**: Replace `portGetTickCnt()`, add proper mutexes for event queue if preemptive.

---

## 12. Critical Implementation Notes

1. **All TX/RX delays are in DW1000 device time units** (40-bit counter, ~15.65 ps per tick). Use `instance_convert_usec_to_devtimeu()` for conversion. The `32h` suffix means high 32 bits of the 40-bit value (i.e., shifted right by 8).

2. **Response is prepared in the ISR callback**, not the application loop. The ISR callback writes the response to the TX buffer and schedules delayed TX. The application loop only processes the result (ToF calculation, state transitions).

3. **`DWT_RESPONSE_EXPECTED`** flag on `dwt_starttx()` automatically enables the receiver after TX completes, which is used for the Poll→Response→Final chain.

4. **The tag calculates `finalTxTime` immediately after receiving the poll TX timestamp**, before any responses arrive. This pre-calculation ensures the Final is sent at a precise, deterministic time regardless of how many responses are received.

5. **`fixedReplyDelayAnc32h`** must be long enough for: preamble + response frame + `RX_RESPONSE_TURNAROUND` (300 μs MCU processing time). It defines the spacing between each anchor's response.

6. **Frame filtering** is configured differently for tag vs anchor:
   - Tag: `DWT_FF_DATA_EN | DWT_FF_ACK_EN` (only data and ACK frames)
   - Anchor: `DWT_FF_NOTYPE_EN` (accept all frame types, including blinks)

7. **The event queue has only 4 slots.** Events are consumed by the application in the main loop between ISR firings. If the application can't keep up, events will be lost. Critical actions (prepare response, schedule TX) happen in the ISR to avoid this.

8. **Anchor A0 eavesdrops on A2's response to A1's poll** (rxResps==3) to collect the A1–A2 ToF without requiring A1 to explicitly report it. This is a bandwidth-saving optimization.

9. **Sleep correction is a signed int16** in milliseconds, ranging from about -0.5×sfPeriod to +1.5×sfPeriod. The tag applies it once and clears it. After the first sync, `tagSleepRnd_ms` is cleared to 0 (no more random jitter).
