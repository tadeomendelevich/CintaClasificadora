#include "servo.h"
#include <avr/io.h>

/**
 * Estructura para cada servo
 */
typedef struct {
    volatile uint8_t* port;  // ej &PORTB
    uint8_t pinBit;          // (1 << PB1)
    // Cuántos overflows mantener en alto (para el pulso)
    uint8_t dutyCycle;       
    // Contador decreciente para apagar el pulso
    uint8_t counter;
} servo_t;

static servo_t servos[SERVO_MAX];
static uint8_t servoCount = 0;

// Este contador lo usamos para contar ~156 overflows => ~20ms
static volatile uint8_t is20ms = SERVO_TIMER_COUNTS_20MS;

void servo_init(void)
{
    servoCount = 0;
    // Podríamos configurar aquí Timer0, pero se puede hacer en main.
}

uint8_t servo_add(volatile uint8_t* port, uint8_t pinBit)
{
    if (servoCount >= SERVO_MAX) {
        return 0xFF; // error
    }
    // Inicializar
    servos[servoCount].port = port;
    servos[servoCount].pinBit = pinBit;
    servos[servoCount].dutyCycle = 10; // ~1ms default
    servos[servoCount].counter = 0;

    // Configurar DDR
    // DDRx = (PORTx - 1). Es una forma simple de llegar a DDR
    volatile uint8_t *ddr = port - 1;
    *ddr |= pinBit; // pin salida
    // Arrancamos en LOW
    *port &= ~pinBit;

    return servoCount++;
}

void servo_setAngle(uint8_t idx, uint8_t angle)
{
    if (idx >= servoCount) return;
    if (angle > 180) angle = 180; // clamp

    // 0° => ~1ms => ~8 ticks
    // 180° => ~2ms => ~16 ticks
    // Ajusta a gusto: 1ms => 8, 2ms => 16 => ~62.5us * 8 => ~500us (depende overflow real).
    // Asegúrate de calibrarlo probando con tu servo.
    uint8_t pulse = 8 + ((angle * 8UL)/180); // range 8..16
    servos[idx].dutyCycle = pulse;
}

void servo_interrupt(void)
{
    // Decrementa contadores: si counter llega a 0 => apaga pin
    for (uint8_t i=0; i<servoCount; i++) {
        if (servos[i].counter > 0) {
            servos[i].counter--;
            if (servos[i].counter == 0) {
                // Apaga pin
                *(servos[i].port) &= ~(servos[i].pinBit);
            }
        }
    }

    // Manejo del período ~20ms
    is20ms--;
    if (!is20ms) {
        // Se reinicia el ciclo
        is20ms = SERVO_TIMER_COUNTS_20MS;
        // Iniciar pulso en cada servo
        for (uint8_t i=0; i<servoCount; i++) {
            servos[i].counter = servos[i].dutyCycle;
            // Enciende pin
            *(servos[i].port) |= servos[i].pinBit;
        }
    }
}
