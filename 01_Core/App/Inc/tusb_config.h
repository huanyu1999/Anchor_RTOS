#ifndef TUSB_CONFIG_H_
#define TUSB_CONFIG_H_

/* ========== MCU ========== */
#define CFG_TUSB_MCU            OPT_MCU_STM32F4     // STM32F405 OTG FS = Synopsys DWC2

/* ========== OS ========== */
#define CFG_TUSB_OS             OPT_OS_FREERTOS      // FreeRTOS OSAL: tud_task() blocks when idle

/* ========== Debug ========== */
#define CFG_TUSB_DEBUG          0

/* ========== Memory ========== */
#define CFG_TUSB_MEM_SECTION                         // no special section needed for STM32F4 OTG FS
#define CFG_TUSB_MEM_ALIGN      TU_ATTR_ALIGNED(4)  // 4-byte align for SDIO DMA compatibility

/* ========== Device stack ========== */
#define CFG_TUD_ENABLED         1
#define BOARD_TUD_RHPORT        0       // OTG FS = port 0 (OTG HS would be port 1)
#define CFG_TUD_ENDPOINT0_SIZE  64

/* ========== MSC ========== */
#define CFG_TUD_MSC             1
#define CFG_TUD_MSC_EP_BUFSIZE  512     // 1 sector per transfer

/* ========== Disable unused classes ========== */
#define CFG_TUD_CDC             0
#define CFG_TUD_HID             0
#define CFG_TUD_MIDI            0
#define CFG_TUD_AUDIO           0
#define CFG_TUD_VENDOR          0

#endif /* TUSB_CONFIG_H_ */
