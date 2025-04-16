#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#define SERVO_MAX  4  // Máximo servos por software
#define SERVO_TIMER_COUNTS_20MS 156  // ~20ms con overflow de 128us

/**
 * @brief Inicializa la librería de servos por software.
 *        Configura las estructuras internas. Debe llamarse antes de servo_add().
 */
void servo_init(void);

/**
 * @brief Registra un nuevo servo en un pin dado.
 * 
 * @param port   Puntero a PORTx, por ejemplo &PORTB
 * @param pinBit bit mask del pin, por ejemplo (1 << PB1)
 * @return Índice del servo si ok, 0xFF si error (máx servos superado).
 */
uint8_t servo_add(volatile uint8_t* port, uint8_t pinBit);

/**
 * @brief Ajusta el ángulo del servo (0..180).
 *        Internamente convertimos a pulsos ~1..2ms.
 *
 * @param idx   índice del servo (retornado por servo_add())
 * @param angle ángulo en grados (0..180)
 */
void servo_setAngle(uint8_t idx, uint8_t angle);

/**
 * @brief Función que se llama en el ISR(TIMER0_OVF_vect). Maneja los pulsos
 *        de todos los servos. Debe invocarse en cada overflow de Timer0.
 */
void servo_interrupt(void);

#endif
