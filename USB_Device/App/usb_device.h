#ifndef USB_DEVICE_H
#define USB_DEVICE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

void MX_USB_Device_Init(void);

#ifdef __cplusplus
}
#endif

#endif
