#ifndef HCSR04_H_
#define HCSR04_H_

#include <avr/io.h>
#include <avr/interrupt.h>

/* ===================== CONFIGURACIÓN ===================== */

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
 * @brief Función que debe llamarse periódicamente para manejar el estado del sensor.
 */
void HCSR04_Task(void);

/**
 * @brief Indica si el sensor está actualmente midiendo.
 * @return 1 si está midiendo, 0 si no.
 */
uint8_t HCSR04_isMeasuring(void);

/**
 * @brief Indica si una nueva medición está disponible.
 * @return 1 si hay nueva medición, 0 si no.
 */
uint8_t HCSR04_isNewMeasurement(void);

/**
 * @brief Obtiene la distancia medida en microsegundos (us).
 * @return Tiempo en microsegundos o 0xFFFFFFFF si no hay nueva medición.
 */
uint32_t HCSR04_getDistance(void);

#endif /* HCSR04_H_ */