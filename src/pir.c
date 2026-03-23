// src/pir.c
#include "pir.h"
#include <avr/io.h>

#define PIR_PIN_bm PIN2_bm   // PD2

void pir_init(void)
{
    // PIR OUT op PD2 als input
    PORTD.DIRCLR = PIR_PIN_bm;

    // Geen pull-up nodig bij meeste PIR modules
    // (ze hebben zelf een driver)
}

uint8_t pir_detected(void)
{
    return (PORTD.IN & PIR_PIN_bm) ? 1 : 0;
}