#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "main.h"

USBD_HandleTypeDef hUsbDeviceFS;

void USBD_Clock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_PeriphCLKInitTypeDef periph = {0};
  RCC_CRSInitTypeDef crs = {0};
  uint32_t n;

  __HAL_RCC_HSI48_ENABLE();
  for (n = 0; n < 2000000U && (RCC->CRRCR & RCC_CRRCR_HSI48RDY) == 0U; n++) {
  }
  if ((RCC->CRRCR & RCC_CRRCR_HSI48RDY) == 0U) {
    Error_Handler();
  }

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  osc.HSI48State = RCC_HSI48_ON;
  osc.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    Error_Handler();
  }

  periph.PeriphClockSelection = RCC_PERIPHCLK_USB;
  periph.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) {
    Error_Handler();
  }

  __HAL_RCC_CRS_CLK_ENABLE();
  crs.Prescaler = RCC_CRS_SYNC_DIV1;
  crs.Source = RCC_CRS_SYNC_SOURCE_USB;
  crs.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000, 1000);
  crs.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  crs.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
  HAL_RCCEx_CRSConfig(&crs);
}

void MX_USB_Device_Init(void)
{
  USBD_Clock_Config();
  if (USBD_Init(&hUsbDeviceFS, &CDC_Desc, DEVICE_FS) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK) {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
    Error_Handler();
  }
  HAL_Delay(2);
  USB->CNTR &= (uint16_t)~(USB_CNTR_PDWN | USB_CNTR_FRES | USB_CNTR_LPMODE | USB_CNTR_FSUSP);
  USB->BCDR |= USB_BCDR_DPPU;
}
