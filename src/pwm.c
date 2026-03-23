// src/pwm.c
#include "pwm.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define LED_PORT PORTD
#define LED_PIN  PIN1_bm

static volatile uint8_t g_duty = 0;  // 0..255
static volatile uint8_t phase  = 0;  // 0..255

ISR(TCD0_OVF_vect)
{
  phase++;
  if (phase < g_duty) LED_PORT.OUTSET = LED_PIN; // aan
  else                LED_PORT.OUTCLR = LED_PIN; // uit
}

void pwm_init(void)
{
  LED_PORT.DIRSET = LED_PIN;
  LED_PORT.OUTCLR = LED_PIN;

  // f_ovf = 32MHz / (1*(PER+1))
  // PER=249 => 128kHz overflow
  // PWM = 128k / 256 = 500Hz (geen zichtbare flikker)
  TCD0.CTRLA = TC_CLKSEL_DIV1_gc;
  TCD0.PER   = 249;

  TCD0.INTCTRLA = TC_OVFINTLVL_MED_gc;
  PMIC.CTRL |= PMIC_MEDLVLEN_bm;
  sei();
}

void pwm_set(uint8_t duty)
{
  g_duty = duty;
}