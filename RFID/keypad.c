// keypad.c
#include "keypad.h"

// If your project already defines delay_ms(...) (it does), use it.
extern void delay_ms(volatile uint32_t ms);

/* ------------ tiny GPIO helpers ------------ */
#define BIT(p)               (1U << (p))
#define GPIO_OUT(port,pin)   do{ (port)->MODER &= ~(3U<<((pin)*2)); (port)->MODER |=  (1U<<((pin)*2)); }while(0)
#define GPIO_IN(port,pin)    do{ (port)->MODER &= ~(3U<<((pin)*2)); }while(0)
#define GPIO_PP(port,pin)    do{ (port)->OTYPER &= ~BIT(pin); }while(0)
#define GPIO_PU(port,pin)    do{ (port)->PUPDR &= ~(3U<<((pin)*2)); (port)->PUPDR |=  (1U<<((pin)*2)); }while(0)
#define GPIO_NOPUPD(port,pin) do{ (port)->PUPDR &= ~(3U<<((pin)*2)); }while(0)
#define GPIO_OSPEED_HI(port,pin) do{ (port)->OSPEEDR |= (3U<<((pin)*2)); }while(0)

#define GPIO_SET(port,pin)   ((port)->BSRR = BIT(pin))
#define GPIO_CLR(port,pin)   ((port)->BSRR = BIT(pin) << 16)
#define GPIO_READ(port,pin)  (((port)->IDR >> (pin)) & 1U)

/* Key map (rows × cols) */
static const char kp_map[4][4] = {
  { '1','2','3','A' },
  { '4','5','6','B' },
  { '7','8','9','C' },
  { '*','0','#','D' }
};

/* Drive all columns HIGH (inactive) */
static inline void cols_release(void) {
  GPIO_SET(KP_C1_PORT, KP_C1_PIN);
  GPIO_SET(KP_C2_PORT, KP_C2_PIN);
  GPIO_SET(KP_C3_PORT, KP_C3_PIN);
  GPIO_SET(KP_C4_PORT, KP_C4_PIN);
}

/* Set exactly one column LOW, others HIGH */
static inline void col_drive(uint8_t idx) {
  // start with all high
  cols_release();
  switch (idx) {
    case 0: GPIO_CLR(KP_C1_PORT, KP_C1_PIN); break;
    case 1: GPIO_CLR(KP_C2_PORT, KP_C2_PIN); break;
    case 2: GPIO_CLR(KP_C3_PORT, KP_C3_PIN); break;
    case 3: GPIO_CLR(KP_C4_PORT, KP_C4_PIN); break;
  }
}

/* Read which row is pulled low; return 0..3, or 0xFF if none */
static inline uint8_t read_row(void) {
  if (!GPIO_READ(KP_R1_PORT, KP_R1_PIN)) return 0;
  if (!GPIO_READ(KP_R2_PORT, KP_R2_PIN)) return 1;
  if (!GPIO_READ(KP_R3_PORT, KP_R3_PIN)) return 2;
  if (!GPIO_READ(KP_R4_PORT, KP_R4_PIN)) return 3;
  return 0xFF;
}

/* ===== Public API ===== */

void keypad_init(void)
{
  /* Enable GPIO clocks for A/B/C */
  RCC->AHB1ENR |= (1U<<0) | (1U<<1) | (1U<<2);

  /* Columns as outputs (idle HIGH), push-pull, high speed, no pull */
  GPIO_OUT(KP_C1_PORT, KP_C1_PIN); GPIO_PP(KP_C1_PORT, KP_C1_PIN); GPIO_OSPEED_HI(KP_C1_PORT, KP_C1_PIN); GPIO_NOPUPD(KP_C1_PORT, KP_C1_PIN);
  GPIO_OUT(KP_C2_PORT, KP_C2_PIN); GPIO_PP(KP_C2_PORT, KP_C2_PIN); GPIO_OSPEED_HI(KP_C2_PORT, KP_C2_PIN); GPIO_NOPUPD(KP_C2_PORT, KP_C2_PIN);
  GPIO_OUT(KP_C3_PORT, KP_C3_PIN); GPIO_PP(KP_C3_PORT, KP_C3_PIN); GPIO_OSPEED_HI(KP_C3_PORT, KP_C3_PIN); GPIO_NOPUPD(KP_C3_PORT, KP_C3_PIN);
  GPIO_OUT(KP_C4_PORT, KP_C4_PIN); GPIO_PP(KP_C4_PORT, KP_C4_PIN); GPIO_OSPEED_HI(KP_C4_PORT, KP_C4_PIN); GPIO_NOPUPD(KP_C4_PORT, KP_C4_PIN);

  /* Rows as inputs with pull-ups */
  GPIO_IN(KP_R1_PORT, KP_R1_PIN); GPIO_PU(KP_R1_PORT, KP_R1_PIN);
  GPIO_IN(KP_R2_PORT, KP_R2_PIN); GPIO_PU(KP_R2_PORT, KP_R2_PIN);
  GPIO_IN(KP_R3_PORT, KP_R3_PIN); GPIO_PU(KP_R3_PORT, KP_R3_PIN);
  GPIO_IN(KP_R4_PORT, KP_R4_PIN); GPIO_PU(KP_R4_PORT, KP_R4_PIN);

  cols_release();
}

/* Debounce helper: confirm the same row stays active for a short time */
static uint8_t confirm_row(uint8_t expect_col, uint8_t *row_out)
{
  for (uint8_t i = 0; i < 4; ++i) {
    col_drive(expect_col);
    delay_ms(2);
    uint8_t r = read_row();
    if (r == 0xFF) return 0;     // nothing now → reject
    if (i == 0) *row_out = r;    // first sample
    else if (r != *row_out) return 0; // unstable
  }
  return 1;
}

char keypad_scan(void)
{
  /* One full scan: drive each column low and see which row goes low */
  for (uint8_t c = 0; c < 4; ++c) {
    col_drive(c);
    delay_ms(1);                   // settle
    uint8_t r = read_row();
    if (r != 0xFF) {
      // debounce / validate
      uint8_t r2 = r;
      if (confirm_row(c, &r2)) {
        // Wait for key release to avoid repeats (simple blocking)
        while (1) {
          col_drive(c);
          delay_ms(2);
          if (read_row() == 0xFF) break;
        }
        cols_release();
        return kp_map[r2][c];
      }
    }
  }

  cols_release();
  return 0;   // no key
}

uint8_t keypad_read_code(char *buf, uint8_t len, uint32_t timeout_ms)
{
  uint32_t elapsed = 0;
  uint8_t n = 0;

  while (n < len && elapsed < timeout_ms) {
    char k = keypad_scan();
    if (k) {
      buf[n++] = k;
      // small gap to avoid bouncing into next read
      delay_ms(30);
      elapsed += 30;
    } else {
      delay_ms(5);
      elapsed += 5;
    }
  }
  return n;
}
