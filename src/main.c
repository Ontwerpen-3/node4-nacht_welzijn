#define F_CPU 32000000UL
#define SAMPLERATE 10000UL
#define SAMPLERATE_OUT 4UL
#define TICKS_PER_SAMPLE (F_CPU / SAMPLERATE)
#define SAMPLES_AVERAGED (SAMPLERATE / SAMPLERATE_OUT)
#define CHANNELS_AVERAGED 4U
#define AVERAGING_GAIN ((uint32_t) SAMPLES_AVERAGED * CHANNELS_AVERAGED)
#define BRIDGE_DRIVE_V 3.3
#define BRIDGE_SENS_V_PER_G (BRIDGE_DRIVE_V * 1e-6)
#define AMP_GAIN 8
#define ADC_FS_V (3.3/2)
#define ADC_RANGE_LSB 2048
#define ADC_V_PER_LSB (ADC_FS_V / ADC_RANGE_LSB)
#define GRAM_PER_LSB (ADC_V_PER_LSB / (BRIDGE_SENS_V_PER_G * AMP_GAIN))

#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "clock.h"
#include "serialF0.h"

#define PORTA_ADCPINS (PIN3_bm | PIN4_bm)
#define PORTD_BRIDGEPOS PIN0_bm
#define PORTD_BRIDGENEG PIN1_bm

// ===== PIR =====
#define PIR_bm   PIN2_bm
#define GREEN_bm PIN3_bm
#define RED_bm   PIN4_bm

// ===== Licht =====
#define LIGHT_ADC_PIN PIN0_bm     // PB0
#define LIGHT_LED_bm  PIN0_bm     // PE0

#define PWM_TOP   255u
#define DUTY_MIN  0u
#define DUTY_MAX  255u

/*
 * ===== LICHT KALIBRATIE =====
 * Startwaardes voor jouw nieuwe fotodiode.
 * Deze moet je later tunen met echte metingen.
 *
 * Idee:
 * - meet ADC in bijna donker
 * - meet ADC rond ~5 lux
 * - meet ADC rond ~50 lux
 * - meet ADC rond ~300 lux
 * - meet ADC bij heel fel licht
 *
 * Verwachting hier:
 * meer licht -> hogere ADC
 */
#define LIGHT_ADC_DARK       80u
#define LIGHT_ADC_5LUX      220u
#define LIGHT_ADC_50LUX     900u
#define LIGHT_ADC_300LUX   2400u
#define LIGHT_ADC_1000LUX  3600u

/*
 * Tot welke lux-waarde moet de lamp actief mee dimmen?
 * Boven deze waarde gaat hij naar uit.
 */
#define LIGHT_ACTIVE_MAX_LUX 300u

static volatile int32_t sAccumulatedSamples;
static volatile uint8_t sReadPhase, sWritePhase;

// ================= LOADCELL CODE =================

static uint8_t ReadCalibrationByte(uint8_t index) {
    uint8_t result;
    NVM.CMD = NVM_CMD_READ_CALIB_ROW_gc;
    result = pgm_read_byte(index);
    NVM.CMD = NVM_CMD_NO_OPERATION_gc;
    return result;
}

static void InitAnalogADC(void) {
    ADCA.CALL = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL0));
    ADCA.CALH = ReadCalibrationByte(offsetof(NVM_PROD_SIGNATURES_t, ADCACAL1));

    PORTA.OUTCLR = PORTA_ADCPINS;
    PORTA.DIRCLR = PORTA_ADCPINS;

    PORTCFG.MPCMASK = PORTA_ADCPINS;
    PORTA.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;

    ADCA.CTRLB = ADC_CURRLIMIT_NO_gc | ADC_RESOLUTION_12BIT_gc | ADC_CONMODE_bm;
    ADCA.REFCTRL = ADC_REFSEL_INTVCC2_gc;
    ADCA.EVCTRL = ADC_SWEEP_0123_gc | ADC_EVSEL_0123_gc | ADC_EVACT_SYNCSWEEP_gc;
    ADCA.PRESCALER = ADC_PRESCALER_DIV32_gc;

    ADCA.CH0.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc;
    ADCA.CH1.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc;
    ADCA.CH2.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc;
    ADCA.CH3.CTRL = ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc;

    ADCA.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc | ADC_CH_MUXNEG_PIN3_gc;
    ADCA.CH1.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc | ADC_CH_MUXNEG_PIN3_gc;
    ADCA.CH2.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc | ADC_CH_MUXNEG_PIN3_gc;
    ADCA.CH3.MUXCTRL = ADC_CH_MUXPOS_PIN4_gc | ADC_CH_MUXNEG_PIN3_gc;

    ADCA.CH3.INTCTRL = ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_HI_gc;
    ADCA.CTRLA = ADC_ENABLE_bm;

    PMIC.CTRL |= PMIC_HILVLEN_bm;
}

static void InitAnalogTimer(void) {
    TCC0.CTRLB = TC_WGMODE_NORMAL_gc;
    TCC0.PER = TICKS_PER_SAMPLE - 1;
    TCC0.CNT = 0;
    TCC0.CTRLA = TC_CLKSEL_DIV1_gc;
    EVSYS.CH0MUX = EVSYS_CHMUX_TCC0_OVF_gc;
}

// ================= LICHT ADC (ADCB op PB0) =================

static void InitLightADC(void) {
    PORTB.DIRCLR = LIGHT_ADC_PIN;
    PORTB.PIN0CTRL = PORT_ISC_INPUT_DISABLE_gc;

    ADCB.REFCTRL = ADC_REFSEL_INTVCC_gc;
    ADCB.CTRLB = ADC_RESOLUTION_12BIT_gc;
    ADCB.PRESCALER = ADC_PRESCALER_DIV64_gc;

    ADCB.CH0.CTRL = ADC_CH_INPUTMODE_SINGLEENDED_gc;
    ADCB.CH0.MUXCTRL = ADC_CH_MUXPOS_PIN0_gc;

    ADCB.CTRLA = ADC_ENABLE_bm;
}

static uint16_t read_light_adc_avg8(void) {
    uint32_t sum = 0;
    for (uint8_t i = 0; i < 8; i++) {
        ADCB.CH0.CTRL |= ADC_CH_START_bm;
        while (!(ADCB.CH0.INTFLAGS & ADC_CH_CHIF_bm));
        ADCB.CH0.INTFLAGS = ADC_CH_CHIF_bm;
        sum += ADCB.CH0RES;
    }
    return (uint16_t)(sum >> 3);
}

// ================= PWM op PE0 via TCE0 =================

static void InitLightPWM(void) {
    PORTE.DIRSET = LIGHT_LED_bm;

    TCE0.CTRLA = TC_CLKSEL_OFF_gc;
    TCE0.CTRLB = TC_WGMODE_SS_gc | TC0_CCAEN_bm;
    TCE0.PER = PWM_TOP;
    TCE0.CCA = 0;
    TCE0.CTRLA = TC_CLKSEL_DIV64_gc;
}

static inline void light_set_duty(uint8_t duty) {
    TCE0.CCA = duty;
}

// ================= LICHT HULPFUNCTIES =================

static uint16_t map_u16(uint16_t x,
                        uint16_t in_min, uint16_t in_max,
                        uint16_t out_min, uint16_t out_max)
{
    if (x <= in_min) return out_min;
    if (x >= in_max) return out_max;

    return (uint16_t)(out_min +
           (((uint32_t)(x - in_min) * (uint32_t)(out_max - out_min)) /
            (uint32_t)(in_max - in_min)));
}

/*
 * ADC -> geschatte lux
 * Piecewise lineair, zodat je later makkelijk echte meetpunten kunt invullen.
 */
static uint16_t adc_to_lux_estimate(uint16_t adc) {
    if (adc <= LIGHT_ADC_DARK) {
        return 0u;
    } else if (adc <= LIGHT_ADC_5LUX) {
        return map_u16(adc, LIGHT_ADC_DARK, LIGHT_ADC_5LUX, 0u, 5u);
    } else if (adc <= LIGHT_ADC_50LUX) {
        return map_u16(adc, LIGHT_ADC_5LUX, LIGHT_ADC_50LUX, 5u, 50u);
    } else if (adc <= LIGHT_ADC_300LUX) {
        return map_u16(adc, LIGHT_ADC_50LUX, LIGHT_ADC_300LUX, 50u, 300u);
    } else if (adc <= LIGHT_ADC_1000LUX) {
        return map_u16(adc, LIGHT_ADC_300LUX, LIGHT_ADC_1000LUX, 300u, 1000u);
    } else {
        return 1000u;
    }
}

/*
 * Lux -> brightness
 *
 * Continue curve:
 * - 0 lux      -> 255
 * - 300 lux    -> 0
 * - daartussen vloeiend en adaptief
 *
 * We gebruiken een kwadratische curve:
 * brightness = (1 - lux/max)^2
 *
 * Daardoor:
 * - in het donker blijft hij mooi fel
 * - bij toenemend omgevingslicht dimt hij steeds vloeiend
 * - geen harde zones / geen aan-uit gevoel
 */
static uint8_t lux_to_brightness(uint16_t lux) {
    uint16_t bright;

    if (lux >= LIGHT_ACTIVE_MAX_LUX) {
        bright = 0u;
    } else {
        uint16_t inv = LIGHT_ACTIVE_MAX_LUX - lux;   // 300..0
        bright = (uint16_t)(((uint32_t)inv * (uint32_t)inv * 255u) /
                            ((uint32_t)LIGHT_ACTIVE_MAX_LUX * (uint32_t)LIGHT_ACTIVE_MAX_LUX));
    }

    if (bright < DUTY_MIN) bright = DUTY_MIN;
    if (bright > DUTY_MAX) bright = DUTY_MAX;

    return (uint8_t)bright;
}

// ================= ADC INTERRUPT =================

ISR(ADCA_CH3_vect) {
    static int32_t sSampleAccumulator;
    static uint16_t sCountdown = SAMPLES_AVERAGED;

    PORTD.OUTTGL = PORTD_BRIDGEPOS | PORTD_BRIDGENEG;

    if (!--sCountdown) {
        if (sReadPhase == sWritePhase) {
            sAccumulatedSamples = sSampleAccumulator;
            sWritePhase++;
        }
        sSampleAccumulator = 0;
        sCountdown = SAMPLES_AVERAGED;
    }

    if (PORTD.IN & PORTD_BRIDGENEG) {
        sSampleAccumulator += (int16_t)ADCA.CH0.RES;
        sSampleAccumulator += (int16_t)ADCA.CH1.RES;
        sSampleAccumulator += (int16_t)ADCA.CH2.RES;
        sSampleAccumulator += (int16_t)ADCA.CH3.RES;
    } else {
        sSampleAccumulator -= (int16_t)ADCA.CH0.RES;
        sSampleAccumulator -= (int16_t)ADCA.CH1.RES;
        sSampleAccumulator -= (int16_t)ADCA.CH2.RES;
        sSampleAccumulator -= (int16_t)ADCA.CH3.RES;
    }
}

// ================= MAIN =================

int main(void) {
    PORTD.OUTSET = PORTD_BRIDGEPOS;
    PORTD.OUTCLR = PORTD_BRIDGENEG;
    PORTD.DIRSET = PORTD_BRIDGEPOS | PORTD_BRIDGENEG;

    init_clock();
    init_stream(F_CPU);

    InitAnalogADC();
    InitAnalogTimer();
    InitLightADC();
    InitLightPWM();

    // PIR
    PORTD.DIRSET = GREEN_bm | RED_bm;
    PORTD.DIRCLR = PIR_bm;
    PORTD.PIN2CTRL = PORT_OPC_PULLDOWN_gc;

    sei();

    uint8_t hi_cnt = 0, lo_cnt = 0, motion = 0;
    uint16_t adc_filt = 0;
    uint8_t light_print_div = 0;
    uint8_t loadcell_print_div = 0;

    while (1) {
        // ===== LOADCELL OUTPUT =====
        while (sReadPhase == sWritePhase);
        int32_t newVal = sAccumulatedSamples;
        sReadPhase = sWritePhase;

        loadcell_print_div++;
        if (loadcell_print_div >= 20) {
            loadcell_print_div = 0;
            printf("%ld -> %.3f avg -> %.3f g\r\n",
                   newVal,
                   (double)newVal / AVERAGING_GAIN,
                   (((double)newVal / AVERAGING_GAIN) * GRAM_PER_LSB));
        }

        // ===== PIR =====
        {
            uint8_t pir = (PORTD.IN & PIR_bm) ? 1 : 0;

            if (pir) {
                if (hi_cnt < 50) hi_cnt++;
                lo_cnt = 0;
            } else {
                if (lo_cnt < 50) lo_cnt++;
                hi_cnt = 0;
            }

            if (!motion && hi_cnt >= 40) motion = 1;
            if ( motion && lo_cnt >= 40) motion = 0;

            if (motion) PORTD.OUTCLR = RED_bm;
            else        PORTD.OUTSET = RED_bm;
        }

        // ===== LICHT =====
        {
            uint16_t adc = read_light_adc_avg8();

            // zelfde rustige filtering als jouw originele code
            adc_filt = adc_filt - (adc_filt >> 3) + (adc >> 3);
            uint16_t a = adc_filt;

            // ADC -> geschatte lux
            uint16_t lux = adc_to_lux_estimate(a);

            // lux -> vloeiende brightness
            uint8_t bright = lux_to_brightness(lux);

            light_set_duty(bright);

            // licht debug sneller tonen
            light_print_div++;
            if (light_print_div >= 10) {
                light_print_div = 0;
                printf("Light ADC:%u  Lux:%u  PWM:%u\r\n", a, lux, bright);
            }
        }

        _delay_ms(1);
    }
}