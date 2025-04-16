#include "ir_sensor.h"

static ir_sensor_t ir_sensors[MAX_IR_SENSORS];
static uint8_t ir_sensor_count = 0;

uint8_t ir_add(volatile uint8_t* pin_register, uint8_t pin_bit) {
	if (ir_sensor_count >= MAX_IR_SENSORS) return 0xFF;

	ir_sensors[ir_sensor_count].pin_register = pin_register;
	ir_sensors[ir_sensor_count].pin_bit = pin_bit;
	ir_sensors[ir_sensor_count].stable_state = 1;  // Suponemos sin caja al inicio
	ir_sensors[ir_sensor_count].previous_state = 1;
	ir_sensors[ir_sensor_count].fall_flag = 0;
	ir_sensors[ir_sensor_count].rise_flag = 0;
	ir_sensors[ir_sensor_count].debounce_counter = 0;

	// Configura como entrada con pull-up
	if (pin_register == &PIND) {
		DDRD &= ~(1 << pin_bit);
		PORTD |= (1 << pin_bit);
		} else if (pin_register == &PINB) {
		DDRB &= ~(1 << pin_bit);
		PORTB |= (1 << pin_bit);
		} else if (pin_register == &PINC) {
		DDRC &= ~(1 << pin_bit);
		PORTC |= (1 << pin_bit);
	}

	return ir_sensor_count++;
}

void ir_update_all(void) {
	for (uint8_t i = 0; i < ir_sensor_count; i++) {
		uint8_t read = (*(ir_sensors[i].pin_register) >> ir_sensors[i].pin_bit) & 0x01;

		if (read != ir_sensors[i].stable_state) {
			ir_sensors[i].debounce_counter++;
			if (ir_sensors[i].debounce_counter >= DEBOUNCE_TICKS) {
				ir_sensors[i].previous_state = ir_sensors[i].stable_state;
				ir_sensors[i].stable_state = read;
				ir_sensors[i].debounce_counter = 0;

				// Detección de flancos
				if (ir_sensors[i].previous_state == 1 && ir_sensors[i].stable_state == 0)
				ir_sensors[i].fall_flag = 1;
				else if (ir_sensors[i].previous_state == 0 && ir_sensors[i].stable_state == 1)
				ir_sensors[i].rise_flag = 1;
			}
			} else {
			ir_sensors[i].debounce_counter = 0;
		}
	}
}

uint8_t ir_fall(uint8_t id) {
	if (id >= ir_sensor_count) return 0;
	uint8_t flag = ir_sensors[id].fall_flag;
	ir_sensors[id].fall_flag = 0;
	return flag;
}

uint8_t ir_rise(uint8_t id) {
	if (id >= ir_sensor_count) return 0;
	uint8_t flag = ir_sensors[id].rise_flag;
	ir_sensors[id].rise_flag = 0;
	return flag;
}

uint8_t ir_state(uint8_t id) {
	if (id >= ir_sensor_count) return 1;
	return ir_sensors[id].stable_state;
}
