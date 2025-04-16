#include "HCSR04.h"

volatile static uint16_t tiempoInicio = 0;
volatile static uint16_t tiempoFin = 0;
volatile static uint16_t pulsoDuracion = 0;
volatile uint8_t nuevaMedicionDisponible = 0;

static uint32_t triggerTimestamp = 0;
static HCSR04_State_t sensorState = HCSR04_IDLE;
static uint32_t echoTimeout = 0;  // Timeout de espera

uint32_t millis(void);

void HCSR04_Init(void) {
	DDRB |= (1 << TRIG);     // TRIG como salida
	DDRB &= ~(1 << ECHO);    // ECHO como entrada

	TCCR1A = 0;
	TCCR1B = (1 << CS11);  // Prescaler 8 ? 0.5us por tick
	TCNT1 = 0;

	TIFR1 = (1 << ICF1);
	TIMSK1 |= (1 << ICIE1);
	TCCR1B |= (1 << ICES1);
}

void HCSR04_Task(void) {
	switch (sensorState) {
		case HCSR04_IDLE:
		PORTB |= (1 << TRIG);
		triggerTimestamp = millis();
		sensorState = HCSR04_TRIGGER_HIGH;
		break;

		case HCSR04_TRIGGER_HIGH:
		if (millis() - triggerTimestamp >= 1) {
			PORTB &= ~(1 << TRIG);
			sensorState = HCSR04_WAIT_FOR_ECHO;
			echoTimeout = millis();  // ?? Iniciamos timeout
		}
		break;

		case HCSR04_WAIT_FOR_ECHO:
		if (millis() - echoTimeout > 30) {  // ? Timeout de 30ms sin respuesta
			sensorState = HCSR04_IDLE;
			TCCR1B |= (1 << ICES1);  // Reiniciar flanco a subida por seguridad
		}
		break;
	}
}


uint8_t HCSR04_isMeasuring(void) {
	return (sensorState == HCSR04_WAIT_FOR_ECHO);
}

uint8_t HCSR04_isNewMeasurement(void) {
	return nuevaMedicionDisponible;
}

uint32_t HCSR04_getDistance(void) {
	if (!nuevaMedicionDisponible) return 0xFFFFFFFF;
	nuevaMedicionDisponible = 0;
	return pulsoDuracion / 2;  // 0.5us por tick
}

ISR(TIMER1_CAPT_vect) {
	if (TCCR1B & (1 << ICES1)) {
		tiempoInicio = ICR1;
		sensorState = HCSR04_WAIT_FOR_ECHO;
		echoTimeout = millis();  //  Registramos tiempo  // Entramos en estado de espera
		TCCR1B &= ~(1 << ICES1);  // Cambiar a flanco de bajada
		} else {
		tiempoFin = ICR1;
		pulsoDuracion = tiempoFin - tiempoInicio;
		nuevaMedicionDisponible = 1;
		sensorState = HCSR04_IDLE;  // Volvemos al estado inicial
		TCCR1B |= (1 << ICES1);  // Volver a flanco de subida
	}
}

