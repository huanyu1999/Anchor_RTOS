#include "app_usb_msc.h"
#include "dev_usbd_desc.h"
#include "dev_usbd_msc_storage.h"
#include "usbd_core.h"
#include "usbd_msc.h"

USBD_HandleTypeDef usbd_device;

void app_usbMscInit(void) {
    USBD_Init(&usbd_device, &MSC_Desc, 0);              /* Init MSC Application */
    USBD_RegisterClass(&usbd_device, USBD_MSC_CLASS);   /* Add Supported Class */
    USBD_MSC_RegisterStorage(&usbd_device, &USBD_MSC_DISK_fops);/* Add Storage callbacks for MSC Class */
    USBD_Start(&usbd_device);                           /* Start Device Process */
}
