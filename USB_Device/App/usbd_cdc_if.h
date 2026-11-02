#ifndef USBD_CDC_IF_H
#define USBD_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_cdc.h"

extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;

/* 0 — байт прочитан, 1 — буфер пуст. Вызывается из основного цикла. */
int cdc_getc(uint8_t *out);
/* 0 — строка ушла, 1 — порт ещё не открыт или передача не успела. */
int cdc_write(const char *s);

#ifdef __cplusplus
}
#endif

#endif
