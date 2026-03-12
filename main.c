// TEAM #3 FALL 2025 — Autonomous Hovercraft
// Main entry point: initializes peripherals and runs the navigation loop.

#include "config.h"
#include "imu.h"
#include "sensors.h"
#include "motion.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

// ── Shared PWM state (used by motion.c) ──────────────────────────────────────
volatile uint8_t thrustDutyCycle = 0;
volatile uint8_t liftDutyCycle   = 0;

static volatile uint8_t  pwmCounter = 0;
static volatile uint32_t g_millis   = 0;

// ── 1 kHz System Tick + Software PWM ─────────────────────────────────────────
ISR(TIMER0_COMPA_vect) {
    g_millis++;
    if (++pwmCounter >= 100) pwmCounter = 0;

    if (pwmCounter < thrustDutyCycle) PORTD |=  (1 << THRUST_FAN_PIN);
    else                              PORTD &= ~(1 << THRUST_FAN_PIN);

    if (pwmCounter < liftDutyCycle)   PORTD |=  (1 << LIFT_FAN_PIN);
    else                              PORTD &= ~(1 << LIFT_FAN_PIN);
}

static void systick_init(void) {
    TCCR0A = (1 << WGM01);               // CTC mode
    TCCR0B = (1 << CS01) | (1 << CS00);  // 64 prescaler
    OCR0A  = 249;                         // 16 MHz / 64 / 250 = 1 kHz
    TIMSK0 = (1 << OCIE0A);
}

uint32_t millis(void) {
    uint8_t s = SREG;
    cli();
    uint32_t m = g_millis;
    SREG = s;
    return m;
}

// ── Entry Point ───────────────────────────────────────────────────────────────
int main(void) {
    systick_init();
    twi_init();
    adc_init();
    servo_init();
    setupFans();

    sei();

    mpu_init();
    calibrate_gyro();

    // Lift on, servo centred, short settle before moving
    runFans(0, LIFT_DUTY);
    set_servo_angle(ANG_MID_DEG);
    _delay_ms(1500);

    while (1) {
        drive_straight_until_wall();
        handle_intersection();
    }
}
