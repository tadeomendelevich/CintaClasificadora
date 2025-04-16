/*
 * util.h
 *
 * Created: 07/04/2025 11:51:15
 *  Author: tadeo
 */ 


#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>

#define MAX_BOXES	8  

typedef enum {
	CATEGORY_NONE = 0,
	CATEGORY_A = 1,
	CATEGORY_B = 2,
	CATEGORY_C = 3
} BoxCategory;

typedef struct {
	BoxCategory buffer[MAX_BOXES];
	uint8_t head;
	uint8_t tail;
	uint8_t count;
} BoxFila;

typedef struct {
	uint8_t esperandoCaja;     // ? antes era variable suelta
	BoxCategory tipoCaja;      // ? tipo de caja detectada
	uint8_t cajaClasificada;
	uint8_t cajaEnIR1;			// Se activa mientras el sensor ir1 está en bajo
	uint8_t cajaEnIR2;			// ? Nueva bandera para el IR2
	uint8_t cajaEnIR3;
	uint8_t indiceCajaEnIR1;
	uint8_t indiceCajaEnIR2;
	uint8_t indiceCajaEnIR3;
	uint8_t ir1_flancoIndex;	// Contador de flancos IR1
	uint8_t ir2_flancoIndex;	// Contador de flancos IR2
	uint8_t ir3_flancoIndex;	// Contador de flancos IR3
} _sCinta;

uint8_t sectorA_flags[MAX_BOXES];

uint8_t sectorB_flags[MAX_BOXES];

uint8_t sectorC_flags[MAX_BOXES];






#endif /* UTIL_H_ */