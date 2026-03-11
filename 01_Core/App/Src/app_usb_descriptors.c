/**
 * @file  app_usb_descriptors.c
 * @brief TinyUSB USB descriptor callbacks — MSC only device.
 *        USB VID/PID 可以按需修改。
 */

#include "tusb.h"
#include <string.h>

/* ============================================================
 *  Device Descriptor
 * ============================================================ */
#define USB_VID     0x0483U     // STMicroelectronics（调试阶段使用，量产时换自己的VID）
#define USB_PID     0x5720U
#define USB_BCD     0x0200U     // USB 2.0

static const tusb_desc_device_t s_desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *)&s_desc_device;
}

/* ============================================================
 *  Configuration Descriptor
 *  Layout: Config | MSC Interface | Bulk-OUT | Bulk-IN
 * ============================================================ */
#define CONFIG_TOTAL_LEN    (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

#define EPNUM_MSC_OUT       0x01U
#define EPNUM_MSC_IN        0x81U

static const uint8_t s_desc_configuration[] = {
    /* Configuration descriptor */
    TUD_CONFIG_DESCRIPTOR(1,            /* bConfigurationValue */
                          1,            /* bNumInterfaces */
                          0,            /* iConfiguration */
                          CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP,
                          100),         /* max power 100 mA */

    /* MSC Interface descriptor + endpoints */
    TUD_MSC_DESCRIPTOR(0,              /* Interface number */
                       4,              /* String index (see string table below) */
                       EPNUM_MSC_OUT,
                       EPNUM_MSC_IN,
                       64),            /* EP packet size (Full-Speed max = 64) */
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void)index;
    return s_desc_configuration;
}

/* ============================================================
 *  String Descriptors
 *  index 0 : Language ID
 *  index 1 : Manufacturer
 *  index 2 : Product
 *  index 3 : Serial Number
 *  index 4 : MSC Interface
 * ============================================================ */
static const char *s_string_desc[] = {
    (const char[]){ 0x09, 0x04 },  /* 0: Language = 0x0409 (English) */
    "HZD Technology",               /* 1: Manufacturer */
    "Anchor eMMC Storage",          /* 2: Product */
    "ANC000001",                    /* 3: Serial number */
    "MSC",                          /* 4: MSC interface string */
};

static uint16_t s_desc_str[32];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void)langid;

    uint8_t chr_count;

    if (index == 0) {
        /* Language ID */
        memcpy(&s_desc_str[1], s_string_desc[0], 2);
        chr_count = 1;
    } else {
        if (index >= (sizeof(s_string_desc) / sizeof(s_string_desc[0]))) {
            return NULL;
        }
        const char *str = s_string_desc[index];
        chr_count = (uint8_t)strlen(str);
        if (chr_count > 31) {
            chr_count = 31;
        }
        for (uint8_t i = 0; i < chr_count; i++) {
            s_desc_str[1 + i] = str[i];  /* ASCII -> UTF-16LE */
        }
    }

    /* First word: length + descriptor type */
    s_desc_str[0] = (uint16_t)((TUSB_DESC_STRING << 8) | (2u * chr_count + 2u));
    return s_desc_str;
}
