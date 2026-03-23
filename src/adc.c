// src/adc.c
#include "adc.h"

#define F_CPU 32000000UL
#include <avr/io.h>
#include <util/delay.h>

static uint16_t adc_read_once(void)
{
    ADCA.CH0.CTRL |= ADC_CH_START_bm;
    while (!(ADCA.CH0.INTFLAGS & ADC_CH_CHIF_bm)) {}
    ADCA.CH0.INTFLAGS = ADC_CH_CHIF_bm;
    return ADCA.CH0RES; // 0..4095
}

void adc_init(void)
{
    // Zorg dat ADCA niet in power reduction staat
    // (op sommige setups staat dit wél aan en dan "doet ADC niks")
    PR.PRPA &= ~PR_ADC_bm;

    // PA3 input
    PORTA.DIRCLR = PIN3_bm;

    // Disable digital input buffer on PA3 (minder ruis)
    PORTA.PIN3CTRL = PORT_ISC_INPUT_DISABLE_gc;

    // Disable ADC while configuring
    ADCA.CTRLA = 0;

    // Referentie: probeer eerst VCC/1.6V (INTVCC) zoals jij had.
    // Als jouw bord dat niet goed doet, switch later naar interne 1V:
    // ADCA.REFCTRL = ADC_REFSEL_INT1V_gc;
    ADCA.REFCTRL = ADC_REFSEL_INTVCC_gc;

    // 12-bit, unsigned
    ADCA.CTRLB = ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm; // CONMODE=1 => unsigned

    // ADC klok: 32MHz/64 = 500kHz (prima)
    ADCA.PRESCALER = ADC_PRESCALER_DIV64_gc;

    // Channel 0: single-ended, gain 1x, input = PA3
    ADCA.CH0.CTRL    = ADC_CH_INPUTMODE_SINGLEENDED_gc | ADC_CH_GAIN_1X_gc;
    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN3_gc;

    // Enable ADC
    ADCA.CTRLA = ADC_ENABLE_bm;
    _delay_ms(10);

    // Dummy read(s) voor settling
    (void)adc_read_once();
    _delay_ms(2);
    (void)adc_read_once();
}

uint16_t adc_read_avg(void)
{
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 8; i++) {
        sum += adc_read_once();
    }
    return (uint16_t)(sum >> 3);
}