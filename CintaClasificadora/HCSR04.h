#ifndef HCSR04_H_
#define HCSR04_H_

#include <avr/io.h>
#include <avr/interrupt.h>

/* ===================== CONFIGURACI�N ===================== */

#define TRIG    PB1     // Pin 9 en Arduino UNO
#define ECHO    PB0     // Pin 8 en Arduino UNO

/* ===================== VARIABLES GLOBALES ===================== */

typedef enum {
    HCSR04_IDLE,
    HCSR04_TRIGGER_HIGH,
    HCSR04_WAIT_FOR_ECHO
} HCSR04_State_t;

extern volatile uint8_t nuevaMedicionDisponible;

/* ===================== PROTOTIPOS ===================== */

/**
 * @brief Inicializa Timer1 y los pines del sensor HC-SR04.
 */
void HCSR04_Init(void);

/**
 * @brief Funci�n que debe llamarse peri�dicamente para manejar el estado del sensor.
 */
void HCSR04_Task(void);

/**
 * @brief Indica si el sensor est� actualmente midiendo.
 * @return 1 si est� midiendo, 0 si no.
 */
uint8_t HCSR04_isMeasuring(void);

/**
 * @brief Indica si una nueva medici�n est� disponible.
 * @return 1 si hay nueva medici�n, 0 si no.
 */
uint8_t HCSR04_isNewMeasurement(void);

/**
 * @brief Obtiene la distancia medida en microsegundos (us).
 * @return Tiempo en microsegundos o 0xFFFFFFFF si no hay nueva medici�n.
 */
uint32_t HCSR04_getDistance(void);

#endif /* HCSR04_H_ */