#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "main.h"
#include <string.h>

#define CDC_RX_RING 512U

static uint8_t rx_packet[CDC_DATA_FS_MAX_PACKET_SIZE];
static uint8_t rx_ring[CDC_RX_RING];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;

static uint8_t tx_buf[256];
static USBD_CDC_LineCodingTypeDef line_coding = {
  115200, 0x00, 0x00, 0x08
};

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t *buf, uint32_t *len);
static int8_t CDC_TransmitCplt_FS(uint8_t *buf, uint32_t *len, uint8_t epnum);

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS = {
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
};

static int8_t CDC_Init_FS(void)
{
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx_packet);
  return (int8_t)USBD_OK;
}

static int8_t CDC_DeInit_FS(void)
{
  return (int8_t)USBD_OK;
}

static int8_t CDC_Control_FS(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
  switch (cmd) {
  case CDC_SET_LINE_CODING:
    if (length >= 7U) {
      line_coding.bitrate = (uint32_t)pbuf[0]
        | ((uint32_t)pbuf[1] << 8)
        | ((uint32_t)pbuf[2] << 16)
        | ((uint32_t)pbuf[3] << 24);
      line_coding.format = pbuf[4];
      line_coding.paritytype = pbuf[5];
      line_coding.datatype = pbuf[6];
    }
    break;
  case CDC_GET_LINE_CODING:
    pbuf[0] = (uint8_t)(line_coding.bitrate);
    pbuf[1] = (uint8_t)(line_coding.bitrate >> 8);
    pbuf[2] = (uint8_t)(line_coding.bitrate >> 16);
    pbuf[3] = (uint8_t)(line_coding.bitrate >> 24);
    pbuf[4] = line_coding.format;
    pbuf[5] = line_coding.paritytype;
    pbuf[6] = line_coding.datatype;
    break;
  default:
    break;
  }
  return (int8_t)USBD_OK;
}

static int8_t CDC_Receive_FS(uint8_t *buf, uint32_t *len)
{
  uint32_t i;
  for (i = 0; i < *len; i++) {
    uint16_t next = (uint16_t)((rx_head + 1U) % CDC_RX_RING);
    if (next != rx_tail) {
      rx_ring[rx_head] = buf[i];
      rx_head = next;
    }
  }
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, rx_packet);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (int8_t)USBD_OK;
}

static int8_t CDC_TransmitCplt_FS(uint8_t *buf, uint32_t *len, uint8_t epnum)
{
  (void)buf;
  (void)len;
  (void)epnum;
  return (int8_t)USBD_OK;
}

int cdc_getc(uint8_t *out)
{
  if (rx_head == rx_tail) {
    return 1;
  }
  *out = rx_ring[rx_tail];
  rx_tail = (uint16_t)((rx_tail + 1U) % CDC_RX_RING);
  return 0;
}

int cdc_write(const char *s)
{
  USBD_CDC_HandleTypeDef *hcdc;
  uint32_t started;
  uint16_t len = 0;

  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED || s == NULL) {
    return 1;
  }
  hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc == NULL) {
    return 1;
  }
  while (s[len] != '\0' && len < (sizeof(tx_buf) - 1U)) {
    len++;
  }
  if (len == 0U) {
    return 0;
  }

  started = HAL_GetTick();
  while (hcdc->TxState != 0U) {
    if ((HAL_GetTick() - started) > 30U) {
      return 1;
    }
  }
  memcpy(tx_buf, s, len);
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, tx_buf, len);
  if (USBD_CDC_TransmitPacket(&hUsbDeviceFS) != USBD_OK) {
    return 1;
  }
  started = HAL_GetTick();
  while (hcdc->TxState != 0U) {
    if ((HAL_GetTick() - started) > 30U) {
      return 1;
    }
  }
  return 0;
}
