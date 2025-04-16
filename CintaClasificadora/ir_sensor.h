#ifndef IR_SENSOR_H_
#define IR_SENSOR_H_

#include <avr/io.h>
#include <stdint.h>

#define MAX_IR_SENSORS 16
#define DEBOUNCE_TICKS 3  // Número de lecturas consecutivas para considerar estable

typedef struct {
	volatile uint8_t* pin_register;  // Dirección de PINx
	uint8_t pin_bit;                 // Bit asociado
	uint8_t stable_state;            // Estado estable (0 o 1)
	uint8_t previous_state;          // Estado anterior
	uint8_t fall_flag;               // 1 si hubo flanco descendente (1?0)
	uint8_t rise_flag;               // 1 si hubo flanco ascendente (0?1)
	uint8_t debounce_counter;        // Cuenta de estabilidad
} ir_sensor_t;


uint8_t ir_add(volatile uint8_t* pin_register, uint8_t pin_bit);
void ir_update_all(void);

uint8_t ir_fall(uint8_t id);
uint8_t ir_rise(uint8_t id);
uint8_t ir_state(uint8_t id);

#endif
