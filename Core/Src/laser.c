#include "laser.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include <stdio.h>
#include <string.h>

#define PIN_CHARGE GPIO_PIN_0
#define PIN_FIRE   GPIO_PIN_1
#define PIN_PEDAL  GPIO_PIN_5
#define PIN_FLOW   GPIO_PIN_6

static uint8_t pol_charge = 0;
static uint8_t pol_fire = 0;
static uint8_t armed;
static uint8_t running;
static uint8_t pedal_down;
static uint8_t pedal_raw;
static uint8_t pedal_run;
static uint8_t watch = 1;
static unsigned freq_hz = 1;
static unsigned charge_ms = 150;
static unsigned fire_ms = 1;
static unsigned level;
static unsigned count_req = 1;
static unsigned shots_done;
static unsigned period_ms = 1000;

enum { ST_IDLE, ST_CHARGE, ST_FIRE, ST_GAP };
static uint8_t state = ST_IDLE;
static uint32_t cycle_start;
static uint32_t mark;
static uint32_t last_rx_ms;
static uint32_t pedal_edge_ms;
static volatile uint32_t flow_pulses;
static uint32_t flow_mark;
static uint32_t flow_x10;
static uint8_t flow_ready;
static uint8_t flow_pending;

static char rx_line[96];
static unsigned rx_len;

static void set_pin(uint16_t pin, uint8_t active_high, uint8_t on)
{
  GPIO_PinState level;
  if (active_high) {
    level = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
  } else {
    level = on ? GPIO_PIN_RESET : GPIO_PIN_SET;
  }
  HAL_GPIO_WritePin(GPIOA, pin, level);
}

static void outputs_off(void)
{
  set_pin(PIN_CHARGE, pol_charge, 0);
  set_pin(PIN_FIRE, pol_fire, 0);
}

static void set_level(unsigned value)
{
  uint32_t low_us;
  uint32_t gap_us;
  if (value > 2000U) {
    value = 2000U;
  }
  low_us = 200U + (value * 200U) / 2000U;
  gap_us = 350U - (value * 220U) / 2000U;
  TIM3->CCR2 = low_us;
  TIM3->ARR = low_us + gap_us - 1U;
  level = value;
}

void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin == PIN_FLOW) {
    flow_pulses++;
  }
}

static int timing_ok(unsigned freq, unsigned charge, unsigned fire)
{
  unsigned period;
  if (freq < 1U || freq > 10U || charge < 1U || charge > 800U || fire < 1U || fire > 20U) {
    return 0;
  }
  period = 1000U / freq;
  return (charge + fire + 5U) <= period;
}

static void start_cycle(uint32_t now)
{
  cycle_start = now;
  mark = now;
  set_pin(PIN_FIRE, pol_fire, 0);
  set_pin(PIN_CHARGE, pol_charge, 1);
  state = ST_CHARGE;
}

static void do_stop(const char *evt)
{
  running = 0;
  pedal_run = 0;
  state = ST_IDLE;
  outputs_off();
  if (evt != NULL) {
    cdc_write(evt);
  }
}

static void send_status(void)
{
  char buf[180];
  unsigned left = 0;
  if (running && count_req > shots_done) {
    left = count_req - shots_done;
  }
  snprintf(buf, sizeof(buf),
           "ok arm=%u run=%u freq=%u charge=%u fire=%u level=%u count=%u left=%u polc=%u polf=%u watch=%u pedal=%u\r\n",
           armed, running, freq_hz, charge_ms, fire_ms, level, count_req, left,
           pol_charge, pol_fire, watch, pedal_down);
  cdc_write(buf);
}

static void handle_line(char *s)
{
  unsigned a;
  unsigned b;
  char which[8];

  while (*s == ' ') {
    s++;
  }
  if (*s == '\0') {
    return;
  }
  if (strcmp(s, "help") == 0) {
    cdc_write("ok cmds: status, arm 0|1, watch 0|1, freq, charge, fire, level, count, pol charge|fire 0|1, run [n], stop, ping\r\n");
    return;
  }
  if (strcmp(s, "status") == 0) {
    send_status();
    return;
  }
  if (strcmp(s, "ping") == 0) {
    cdc_write("ok\r\n");
    return;
  }
  if (strcmp(s, "stop") == 0) {
    do_stop(NULL);
    cdc_write("ok\r\n");
    return;
  }
  if (sscanf(s, "arm %u", &a) == 1 && a <= 1U) {
    if (running) {
      cdc_write("err busy\r\n");
      return;
    }
    armed = (uint8_t)a;
    cdc_write("ok\r\n");
    return;
  }
  if (sscanf(s, "watch %u", &a) == 1 && a <= 1U) {
    watch = (uint8_t)a;
    cdc_write("ok\r\n");
    return;
  }
  if (sscanf(s, "freq %u", &a) == 1) {
    if (running || a < 1U || a > 10U) {
      cdc_write("err freq\r\n");
      return;
    }
    freq_hz = a;
    period_ms = 1000U / freq_hz;
    cdc_write("ok\r\n");
    return;
  }
  if (sscanf(s, "charge %u", &a) == 1) {
    if (running || a < 1U || a > 800U) {
      cdc_write("err charge\r\n");
      return;
    }
    charge_ms = a;
    cdc_write("ok\r\n");
    return;
  }
  if (sscanf(s, "fire %u", &a) == 1) {
    if (running || a < 1U || a > 20U) {
      cdc_write("err fire\r\n");
      return;
    }
    fire_ms = a;
    cdc_write("ok\r\n");
    return;
  }
  if (sscanf(s, "level %u", &a) == 1 && a <= 2000U) {
    set_level(a);
    cdc_write("ok\r\n");
    return;
  }
  if (sscanf(s, "count %u", &a) == 1) {
    if (running) {
      cdc_write("err busy\r\n");
      return;
    }
    count_req = a;
    cdc_write("ok\r\n");
    return;
  }
  if (sscanf(s, "pol %7s %u", which, &b) == 2 && b <= 1U) {
    if (running) {
      cdc_write("err busy\r\n");
      return;
    }
    if (strcmp(which, "charge") == 0) {
      pol_charge = (uint8_t)b;
    } else if (strcmp(which, "fire") == 0) {
      pol_fire = (uint8_t)b;
    } else {
      cdc_write("err pol\r\n");
      return;
    }
    outputs_off();
    cdc_write("ok\r\n");
    return;
  }
  if (strcmp(s, "run") == 0 || sscanf(s, "run %u", &a) == 1) {
    if (s[3] != '\0') {
      count_req = a;
    }
    if (!armed) {
      cdc_write("err not armed\r\n");
      return;
    }
    if (!timing_ok(freq_hz, charge_ms, fire_ms)) {
      cdc_write("err timing\r\n");
      return;
    }
    shots_done = 0;
    period_ms = 1000U / freq_hz;
    running = 1;
    last_rx_ms = HAL_GetTick();
    start_cycle(last_rx_ms);
    cdc_write("ok\r\n");
    return;
  }
  cdc_write("err unknown\r\n");
}

static void usb_rx_poll(void)
{
  uint8_t byte;
  while (cdc_getc(&byte) == 0) {
    last_rx_ms = HAL_GetTick();
    if (byte == '\r') {
      continue;
    }
    if (byte == '\n') {
      rx_line[rx_len] = '\0';
      rx_len = 0;
      handle_line(rx_line);
      continue;
    }
    if (rx_len + 1U < sizeof(rx_line)) {
      rx_line[rx_len++] = (char)byte;
    } else {
      rx_len = 0;
    }
  }
}

static void hw_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  gpio.Pin = PIN_CHARGE | PIN_FIRE;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);
  outputs_off();

  gpio.Pin = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &gpio);

  /* 16 MHz / 16 = 1 us. Active-low PWM: low for CCR2, then high until ARR. */
  TIM3->PSC = 15;
  TIM3->CCMR1 = TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2PE;
  TIM3->CCER = TIM_CCER_CC2E | TIM_CCER_CC2P;
  set_level(0);
  TIM3->CR1 = TIM_CR1_ARPE;
  TIM3->EGR = TIM_EGR_UG;
  TIM3->CR1 |= TIM_CR1_CEN;

  gpio.Pin = PIN_PEDAL;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = 0;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = PIN_FLOW;
  gpio.Mode = GPIO_MODE_IT_RISING;
  gpio.Pull = GPIO_PULLDOWN;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = 0;
  HAL_GPIO_Init(GPIOA, &gpio);
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 7, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void laser_init(void)
{
  hw_init();
  last_rx_ms = HAL_GetTick();
  flow_mark = last_rx_ms;
}

static void flow_poll(uint32_t now)
{
  char buf[24];
  uint32_t n;
  uint32_t dt;

  if ((now - flow_mark) >= 1000U) {
    dt = now - flow_mark;
    flow_mark = now;
    __disable_irq();
    n = flow_pulses;
    flow_pulses = 0U;
    __enable_irq();
    flow_x10 = (n * 1000U) / dt;
    flow_ready = 1U;
    flow_pending = 1U;
  }
  if (!flow_pending || state == ST_CHARGE || state == ST_FIRE) {
    return;
  }
  snprintf(buf, sizeof(buf), "flow %u.%u\r\n",
           (unsigned)(flow_x10 / 10U), (unsigned)(flow_x10 % 10U));
  if (cdc_write(buf) == 0) {
    flow_pending = 0U;
  }
}

static void pedal_poll(uint32_t now)
{
  uint8_t down = (uint8_t)(HAL_GPIO_ReadPin(GPIOA, PIN_PEDAL) == GPIO_PIN_RESET);
  if (down != pedal_raw) {
    pedal_raw = down;
    pedal_edge_ms = now;
    return;
  }
  if (down == pedal_down || (now - pedal_edge_ms) < 30U) {
    return;
  }
  pedal_down = down;
  if (down) {
    if (!armed || running || !timing_ok(freq_hz, charge_ms, fire_ms)) {
      return;
    }
    shots_done = 0;
    period_ms = 1000U / freq_hz;
    running = 1;
    pedal_run = 1;
    last_rx_ms = now;
    start_cycle(now);
    cdc_write("evt pedal\r\n");
    return;
  }
  if (pedal_run) {
    do_stop("evt pedal off\r\n");
  }
}

void laser_poll(void)
{
  uint32_t now;

  usb_rx_poll();
  now = HAL_GetTick();
  flow_poll(now);
  pedal_poll(now);
  if (running && watch && !pedal_run && (now - last_rx_ms) > 1500U) {
    do_stop("evt stop timeout\r\n");
    return;
  }
  if (!running) {
    return;
  }

  switch (state) {
  case ST_CHARGE:
    if ((now - mark) >= charge_ms) {
      set_pin(PIN_CHARGE, pol_charge, 0);
      set_pin(PIN_FIRE, pol_fire, 1);
      mark = now;
      state = ST_FIRE;
    }
    break;
  case ST_FIRE:
    if ((now - mark) >= fire_ms) {
      set_pin(PIN_FIRE, pol_fire, 0);
      state = ST_GAP;
    }
    break;
  case ST_GAP:
    if ((now - cycle_start) >= period_ms) {
      shots_done++;
      if (count_req != 0U && shots_done >= count_req) {
        running = 0;
        pedal_run = 0;
        state = ST_IDLE;
        outputs_off();
        cdc_write("evt done\r\n");
      } else {
        start_cycle(now);
      }
    }
    break;
  default:
    outputs_off();
    running = 0;
    break;
  }
}
