/* Includes ------------------------------------------------------------------*/
/**
 * @file main.c
 * @brief Sistema de clasificación de cajas con servomotores y sensores IR usando ATmega328P.
 * 
 * Este archivo implementa la lógica principal de un sistema automático de clasificación de objetos en una cinta 
 * transportadora. Utiliza sensores infrarrojos, un sensor ultrasónico HC-SR04 y servomotores para clasificar 
 * objetos según su altura. La comunicación con el exterior se realiza mediante USART, con un protocolo binario 
 * personalizado. El sistema incluye múltiples tareas temporizadas e interrupciones.
 * 
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <string.h> 
#include "HCSR04.h"
#include "servo.h"
#include "ir_sensor.h"
#include "util.h"



/* END Includes --------------------------------------------------------------*/

/* typedef -------------------------------------------------------------------*/
/**
 * @union _uflag1
 * @brief Bandera de control general del sistema.
 */
typedef union {
	struct {
		uint8_t is10ms : 1;     // Bandera para indicar que pasó 10ms
		uint8_t isNewState : 1; // Bandera para indicar nueva medición
		uint8_t lastState : 1;  // Último estado del sensor
		uint8_t measuring : 1;  // Bandera para indicar si estamos midiendo
		uint8_t sendAllSensorsFlag : 1; // Bandera para enviar todos los datos en
		uint8_t cintaDetectando : 1;     // Se detectó caja (IR0 en alto)
		uint8_t cintaPromReady : 1;      // Promedio listo para usar
		uint8_t reserved : 1;         // Bits que sobran
	} bit;
	uint8_t byte;
} _uflag1;
/**
 * @union _uflag2
 * @brief Bandera de control para el estado de los servomotores.
 */
typedef union {
	struct {
		uint8_t servoAPateo : 1;
		uint8_t servoAReturning : 1;  
		uint8_t servoBPateo : 1;
		uint8_t servoBReturning : 1; 
		uint8_t servoCPateo : 1;
		uint8_t servoCReturning : 1; 
		uint8_t reserved : 2;
	} bit;
	uint8_t byte;
} _uflag2;


/**
 * @union _uWord
 * @brief Unión para conversión y acceso byte/word/dword a una misma variable.
 */
typedef union{
    uint32_t    ui32;
    int32_t     i32;
    uint16_t    ui16[2];
    int16_t     i16[2];
    uint8_t     ui8[4];
    int8_t      i8[4];
}_uWord;

/**
 * @struct _sRx
 * @brief Estructura para recepción de datos USART con protocolo tipo buffer circular.
 */
typedef struct{
	uint8_t *buff;      //!< Puntero para el buffer de recepción
	uint8_t indexR;     //!< Índice de lectura del buffer circular
	uint8_t indexW;     //!< Índice de escritura del buffer circular
	uint8_t indexData;  //!< Índice para identificar la posición del dato
	uint8_t mask;       //!< Máscara para controlar el tamaño del buffer
	uint8_t chk;        //!< Variable para calcular el checksum
	uint8_t nBytes;     //!< Número de bytes recibidos
	uint8_t header;     //!< Estado de la máquina de estados del protocolo
	uint8_t timeOut;    //!< Timeout para resetear la MEF
	uint8_t isComannd;  //!< Bandera de comando recibido
}_sRx;


/**
 * @struct _sTx
 * @brief Estructura para transmisión de datos USART con protocolo tipo buffer circular.
 */
typedef struct{
    uint8_t *buff;      //!< Puntero para el buffer de transmisión*
    uint8_t indexR;     //!<indice de lectura del buffer circular
    uint8_t indexW;     //!<indice de escritura del buffer circular
    uint8_t mask;       //!<máscara para controlar el tamaño del buffer
    uint8_t chk;        //!< variable para calcular el checksum
}_sTx;

/**
 * @brief Enumeración para la maquina de estados
 * que se encarga de decodificar el protocolo
 * de comunicación
 *  
 */
typedef enum{
    HEADER_U,
    HEADER_N,
    HEADER_E,
    HEADER_R,
    NBYTES,
    TOKEN,
    PAYLOAD
}_eDecode;

/**
 * @enum _eCmd
 * @brief Comandos del protocolo de comunicación serie.
 */
typedef enum{
    ALIVE = 0xF0,
    FIRMWARE= 0xF1,
    LEDSTATUS = 0x10,
    BUTTONSTATUS = 0x12,
    ANALOGSENSORS = 0xA0,
    SETBLACKCOLOR = 0xA6,
    SETWHITECOLOR = 0xB1,
    SETLINESPEED = 0xA7,
    MOTORTEST = 0xA1,
    SERVOANGLE = 0xA2,
    CONFIGSERVO = 0xA5,
    SERVOFINISHMOVE = 0x0A,
    GETDISTANCE = 0xA3,
    GETSPEED = 0xA4,
    SENDALLSENSORS = 0xA9,
	STOPALLSENSORS = 0xAA,
    RADAR = 0xA8,
    SW0 = 0xB2,
	BOXCATEGORY = 0xB3,
	SERVOAPATEO = 0xB4,
	SERVOBPATEO = 0xB5,
	SERVOCPATEO = 0xB6,
	GETALLBOXES = 0xB7,
	CAJASPATEADAS = 0xB8, 
    ACK = 0x0D,
    UNKNOWN = 0xFF
}_eCmd;
/**
 * @struct _delay_t
 * @brief Estructura para control de delays no bloqueantes.
 */
typedef struct {
	uint32_t startTime;
	uint16_t interval;
	uint8_t isRunning;
}_delay_t;

/* END typedef ---------------------------------------------------------------*/

/* define --------------------------------------------------------------------*/
#define BAUD				115200
#define MYUBRR				((F_CPU / (8UL * BAUD)) - 1)  // ? (16000000 / (8 * 115200)) - 1 = 16

#define RXBUFSIZE           256
#define TXBUFSIZE           256

#define LASTSTATESW0		flag1.bit.lastState
#define IS10MS				flag1.bit.is10ms
#define ISNEWSTATESW0		flag1.bit.isNewState
#define MEASURING			flag1.bit.measuring

#define LED					PB5   // Pin del LED en Arduino UNO (PB5 es el LED L en el pin 13)

#define ALTURAHCSR			20


#define ALTURAMAXIMA_A		16
#define ALTURAMINIMA_A		13

#define ALTURAMAXIMA_B		12
#define ALTURAMINIMA_B		11

#define ALTURAMAXIMA_C		11
#define ALTURAMINIMA_C		8

//if (distancia_cm > ALTURAMINIMA_A && distancia_cm <= ALTURAMAXIMA_A) tipo = CATEGORY_A; // CATEGORY_A: (13, 16]
//else if (distancia_cm > ALTURAMINIMA_B && distancia_cm <= ALTURAMAXIMA_B) tipo = CATEGORY_B; // CATEGORY_B: (11, 13]
//else if (distancia_cm > ALTURAMINIMA_C && distancia_cm <= ALTURAMAXIMA_C) tipo = CATEGORY_C; // CATEGORY_C: (8, 11]

#define SERVO_PIN		PD7
#define SERVO2_PIN		PB4
#define SERVO3_PIN		PB3

/* END define ----------------------------------------------------------------*/


/* Function prototypes -------------------------------------------------------*/

/**
 * @brief Ejecuta las tareas del puerto serie Decodificación/trasnmisión
 * 
 * @param dataRx Estructura de datos de la recepción
 * @param dataTx Estructura de datos de la trasnmisión
 * @param source Identifica la fuente desde donde se enviaron los datos
 */
void serialTask(_sRx *dataRx, _sTx *dataTx, uint8_t source);

/**
 * @brief Recepción de datos por el puerto serie
 * 
 */
void onRxData();

/**
 * @brief Pone el encabezado del protocolo, el ID y la cantidad de bytes a enviar
 * 
 * @param dataTx Estructura para la trasnmisión de datos
 * @param ID Identificación del comando que se envía
 * @param frameLength Longitud de la trama del comando
 * @return uint8_t devuelve el Checksum de los datos agregados al buffer de trasnmisión
 */
uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength);

/**
 * @brief Agrega un byte al buffer de transmisión
 * 
 * @param dataTx Estructura para la trasnmisión de datos
 * @param byte El elemento que se quiere agregar
 * @return uint8_t devuelve el Checksum del dato agregado al buffer de trasnmisión
 */
uint8_t putByteOnTx(_sTx    *dataTx, uint8_t byte);

/**
 * @brief Agrega un String al buffer de transmisión
 * 
 * @param dataTx Estructura para la trasnmisión de datos
 * @param str String a agregar
 * @return uint8_t devuelve el Checksum del dato agregado al buffer de trasnmisión
 */
uint8_t putStrOntx(_sTx *dataTx, const char *str);

uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos);

/**
 * @brief Decodifica la trama recibida
 * 
 * @param dataRx Estructura para la recepción de datos
 */
void decodeHeader(_sRx *dataRx);

/**
 * @brief Decodifica el comando recibido en la transmisión y ejecuita las tareas asociadas a dicho comando
 * 
 * @param dataRx Estructura para la recepción de datos
 * @param dataTx Estructura para la trasnmisión de datos
 */
void decodeCommand(_sRx *dataRx, _sTx *dataTx);

/**
 * @brief Rellena el buffer de transmisión con datos relacionados al encabezado recibido.
 * 
 * @param header Identificador del comando recibido.
 */
void FillPayload(uint8_t header);
/**
 * @brief Inicializa la estructura de delay no bloqueante.
 * 
 * @param delay Puntero a la estructura de delay.
 * @param interval Intervalo de tiempo deseado en milisegundos.
 */
void delayConfig(_delay_t *delay, uint16_t interval);
/**
 * @brief Modifica y reinicia el intervalo de un delay no bloqueante.
 * 
 * @param delay Puntero a la estructura del delay.
 * @param interval Nuevo tiempo de intervalo en milisegundos.
 */
void delayWrite(_delay_t *delay, uint16_t interval);
/**
 * @brief Verifica si el intervalo del delay ha transcurrido.
 * 
 * @param delay Puntero a la estructura del delay.
 * @return uint8_t 1 si el tiempo se ha cumplido, 0 en caso contrario.
 */
uint8_t delayRead(_delay_t *delay);  
/**
 * @brief Ejecuta las tareas principales del sistema de cinta transportadora.
 * 
 * Incluye la clasificación de cajas, detección por sensores IR, y activación de servos.
 */
void cintaTask();
/**
 * @brief Retorna el tiempo actual en milisegundos desde el inicio del sistema.
 * 
 * @return uint32_t Tiempo actual en milisegundos.
 */
uint32_t millis();
/**
 * @brief Verifica si la cola de cajas está vacía.
 * 
 * @param f Puntero a la estructura de la fila de cajas.
 * @return uint8_t 1 si está vacía, 0 si contiene elementos.
 */
uint8_t BoxFila_isEmpty(BoxFila *f);
/**
 * @brief Elimina y devuelve la primera caja de la cola (FIFO).
 * 
 * @return BoxCategory Categoría de la caja eliminada. Si está vacía, devuelve CATEGORY_NONE.
 */
BoxCategory BoxFila_popAll();
/**
 * @brief Inicializa la cola de cajas.
 * 
 * @param f Puntero a la estructura de la fila de cajas.
 */
void BoxFila_init(BoxFila *f);
/**
 * @brief Verifica si la cola de cajas está llena.
 * 
 * @param f Puntero a la estructura de la fila de cajas.
 * @return uint8_t 1 si está llena, 0 si hay espacio disponible.
 */
uint8_t BoxFila_isFull(BoxFila *f);
/**
 * @brief Agrega una nueva caja a la cola.
 * 
 * @param f Puntero a la estructura de la fila de cajas.
 * @param category Categoría de la caja a insertar.
 * @return uint8_t 1 si se insertó correctamente.
 */
uint8_t BoxFila_push(BoxFila *f, BoxCategory category);

/**
 * @brief Envía por USART el contenido actual de la fila de cajas.
 */
BoxCategory BoxFila_popAll();
/**
 * @brief Envía por USART la categoría de la caja clasificada más recientemente.
 * 
 * @param tipo Categoría de la caja.
 */
void enviarBoxFila();
/**
 * @brief Verifica si hay una nueva caja ingresando en la cinta transportadora.
 */
void enviarCategoria(BoxCategory tipo);

/**
 * @brief Detecta si una caja tipo A debe ser empujada por el servomotor correspondiente.
 */
void verificarEsperarNuevaCaja();
/**
 * @brief Verifica si el servo A debe volver a su posición original tras empujar una caja.
 */
void verificarPateoCajaTipoA();
/**
 * @brief Detecta si la caja tipo A ha salido del área del sensor tras ser empujada.
 */
void verificarRetornoServoA();
/**
 * @brief Detecta si una caja tipo B debe ser empujada por el servomotor correspondiente.
 */
void verificarSalidaCajaTipoA();
/**
 * @brief Verifica si el servo B debe volver a su posición original tras empujar una caja.
 */
void verificarPateoCajaTipoB();
/**
 * @brief Detecta si la caja tipo B ha salido del área del sensor tras ser empujada.
 */
void verificarRetornoServoB();
/**
 * @brief Detecta si una caja tipo C debe ser empujada por el servomotor correspondiente.
 */
void verificarSalidaCajaTipoB();
/**
 * @brief Verifica si el servo C debe volver a su posición original tras empujar una caja.
 */
void verificarPateoCajaTipoC();
/**
 * @brief Detecta si la caja tipo C ha salido del área del sensor tras ser empujada.
 */
void verificarRetornoServoC();
/**
 * @brief Ejecuta la lógica de detección y clasificación de la cinta transportadora.
 * 
 * Controla sensores IR, clasificación de cajas por distancia, y activación de servomotores.
 */
void cintaTask();
/* END Function prototypes ---------------------------------------------------*/


/* Global variables ----------------------------------------------------------*/
volatile _uflag1 flag1;

volatile _uflag2 flag2;

_sTx dataTx;

volatile uint8_t buffRx[RXBUFSIZE];

uint8_t buffTx[TXBUFSIZE];

volatile _sRx dataRx;

uint8_t globalIndex, index2;

_delay_t generalTime; 

_delay_t oneSecond;

_delay_t allSensorsTime;

const char firmware[] = "EX100923v01\n";

_uWord myWord;

volatile uint32_t timer_millis = 0;  // Contador global de milisegundos

uint32_t distancia;  // Distancia medida por el HCSR04 en centimetros

uint8_t buttonMode = 0;

volatile uint8_t servo_angle = 0;           // Ángulo actual del servo

volatile uint8_t direction = 1;             // Dirección del barrido (1 = sube, 0 = baja)

volatile uint16_t servo_tick_count = 0;     // Contador de ticks para el temporizador de 500ms

uint8_t servoA;

uint8_t servoB;

uint8_t servoC;

uint8_t ir0;

uint8_t ir1;

uint8_t ir2;

uint8_t ir3;

_sCinta cinta;

_delay_t timeoutServoA;

_delay_t timeoutServoB;

_delay_t timeoutServoC;

_delay_t ir1Timeout;

_delay_t ir2Timeout;

_delay_t ir3Timeout;

BoxFila boxFila;

uint8_t contadorCajasPateadas;

uint8_t clasificacionListaParaPateo; 

/* END Global variables ------------------------------------------------------*/


/* Function ISR --------------------------------------------------------------*/

ISR(TIMER0_OVF_vect)
{
	// Llamamos a la libreria servo
	servo_interrupt();
}

ISR(TIMER1_COMPB_vect) {  // Interrupción cada 10ms
	IS10MS = 1;
	OCR1B += 20000;  // Siguiente interrupción en 10ms
}

ISR(TIMER2_COMPA_vect) {
	timer_millis++;
}

ISR(USART_RX_vect) {
	uint8_t byte = UDR0;
	dataRx.buff[dataRx.indexW++] = byte;
	dataRx.indexW &= dataRx.mask;

	// (Opcional) Ver el byte:
	// putByteOnTx(&dataTx, byte);

	PORTB ^= (1 << PB5);
}

/* END Function ISR ----------------------------------------------------------*/

/* Function prototypes user code ----------------------------------------------*/

void timer0_init(void)
{
	TCCR0A = 0x00;               // Modo Normal
	TCCR0B = (1 << CS01);        // Prescaler = 8 -> 0.5 us por tick
	TCNT0 = 0;                   // Timer a 0
	TIMSK0 = (1 << TOIE0);       // Habilitar interrupción por overflow
}


void Timer2_Init(void) {
	TCCR2A = (1 << WGM21);              // CTC
	TCCR2B = (1 << CS22);               // Prescaler = 64
	OCR2A = 249;
	TIMSK2 |= (1 << OCIE2A);
}

void serialTask(_sRx *dataRx, _sTx *dataTx, uint8_t source)
{
    if(dataRx->isComannd){
        dataRx->isComannd=0;
        decodeCommand(dataRx,dataTx);
    }

    if(delayRead(&generalTime)){
        if(dataRx->header){
            dataRx->timeOut--;
        if(!dataRx->timeOut)
            dataRx->header = HEADER_U;
        }
    }

    if(dataRx->indexR!=dataRx->indexW){
        decodeHeader(dataRx);
       /* CODIGO A EFECTOS DE EVALUAR SI FUNCIONA LA RECEPCIÓN , SE DEBE DESCOMENTAR 
       Y COMENTAR LA LINEA decodeHeader(dataRx); 
       while (dataRx->indexR!=dataRx->indexW){
            dataTx->buff[dataTx->indexW++]=dataRx->buff[dataRx->indexR++];
            dataTx->indexW &= dataTx->mask;
            dataRx->indexR &= dataRx->mask;
        } */
    }
        

    if (dataTx->indexR != dataTx->indexW) {
	    // Solo usar USART (source == 0)
	    if (!source) {
		    if (UCSR0A & (1 << UDRE0)) {
			    UDR0 = dataTx->buff[dataTx->indexR++];
			    dataTx->indexR &= dataTx->mask;
		    }
	    }
    }
}

void onRxData(void) {
	while (UCSR0A & (1 << RXC0)) {
		uint8_t byte = UDR0;
		dataRx.buff[dataRx.indexW++] = byte;
		dataRx.indexW &= dataRx.mask;
	}
}

uint8_t putHeaderOnTx(_sTx  *dataTx, _eCmd ID, uint8_t frameLength)
{
	dataTx->chk = 0;
	dataTx->buff[dataTx->indexW++]='U';
	dataTx->indexW &= dataTx->mask;
	dataTx->buff[dataTx->indexW++]='N';
	dataTx->indexW &= dataTx->mask;
	dataTx->buff[dataTx->indexW++]='E';
	dataTx->indexW &= dataTx->mask;
	dataTx->buff[dataTx->indexW++]='R';
	dataTx->indexW &= dataTx->mask;
	dataTx->buff[dataTx->indexW++]=frameLength+1;
	dataTx->indexW &= dataTx->mask;
	dataTx->buff[dataTx->indexW++]=':';
	dataTx->indexW &= dataTx->mask;
	dataTx->buff[dataTx->indexW++]=ID;
	dataTx->indexW &= dataTx->mask;
	dataTx->chk ^= (frameLength+1);
	dataTx->chk ^= ('U' ^'N' ^'E' ^'R' ^ID ^':') ;
	return  dataTx->chk;
}

uint8_t putByteOnTx(_sTx *dataTx, uint8_t byte) {
	dataTx->buff[dataTx->indexW++] = byte;
	dataTx->indexW &= dataTx->mask;
	dataTx->chk ^= byte;
	return dataTx->chk;
}

uint8_t putStrOntx(_sTx *dataTx, const char *str)
{
	globalIndex=0;
	while(str[globalIndex]){
		dataTx->buff[dataTx->indexW++]=str[globalIndex];
		dataTx->indexW &= dataTx->mask;
		dataTx->chk ^= str[globalIndex++];
	}
	return dataTx->chk ;
}

uint8_t getByteFromRx(_sRx *dataRx, uint8_t iniPos, uint8_t finalPos){
	uint8_t getByte;
	dataRx->indexData += iniPos;
	dataRx->indexData &=dataRx->mask;
	getByte = dataRx->buff[dataRx->indexData];
	dataRx->indexData += finalPos;
	dataRx->indexData &=dataRx->mask;
	return getByte;
}

void decodeHeader(_sRx *dataRx) {
	uint8_t auxIndex = dataRx->indexW;
	uint8_t max_iter = 64;

	while (dataRx->indexR != auxIndex && max_iter--) {
		switch (dataRx->header) {
			case HEADER_U:
			if (dataRx->buff[dataRx->indexR] == 'U') {
				dataRx->header = HEADER_N;
				dataRx->timeOut = 5;
			}
			break;

			case HEADER_N:
			if (dataRx->buff[dataRx->indexR] == 'N') {
				dataRx->header = HEADER_E;
				} else {
				dataRx->header = HEADER_U;
				dataRx->indexR--;
			}
			break;

			case HEADER_E:
			if (dataRx->buff[dataRx->indexR] == 'E') {
				dataRx->header = HEADER_R;
				} else {
				dataRx->header = HEADER_U;
				dataRx->indexR--;
			}
			break;

			case HEADER_R:
			if (dataRx->buff[dataRx->indexR] == 'R') {
				dataRx->header = NBYTES;
				} else {
				dataRx->header = HEADER_U;
				dataRx->indexR--;
			}
			break;

			case NBYTES:
			dataRx->nBytes = dataRx->buff[dataRx->indexR];
			dataRx->header = TOKEN;
			break;

			case TOKEN:
			if (dataRx->buff[dataRx->indexR] == ':') {
				dataRx->header = PAYLOAD;
				dataRx->indexData = dataRx->indexR + 1;
				dataRx->indexData &= dataRx->mask;
				dataRx->chk = 'U' ^ 'N' ^ 'E' ^ 'R' ^ dataRx->nBytes ^ ':';
				} else {
				dataRx->header = HEADER_U;
				dataRx->indexR--;
			}
			break;

			case PAYLOAD:
			dataRx->nBytes--;
			if (dataRx->nBytes > 0) {
				dataRx->chk ^= dataRx->buff[dataRx->indexR];
				} else {
				dataRx->header = HEADER_U;
				if (dataRx->buff[dataRx->indexR] == dataRx->chk) {
					dataRx->isComannd = 1;
				}
			}
			break;

			default:
			dataRx->header = HEADER_U;
			break;
		}
		dataRx->indexR++;
		dataRx->indexR &= dataRx->mask;
	}
}

void decodeCommand(_sRx *dataRx, _sTx *dataTx) {
	uint8_t angleSource;
	switch (dataRx->buff[dataRx->indexData]) {
		case ALIVE:
		putHeaderOnTx(dataTx, ALIVE, 2);     // payload de 1 byte: sólo el ACK
		putByteOnTx(dataTx, ACK);            // este valor entra en el checksum
		putByteOnTx(dataTx, dataTx->chk);    // este es el checksum final
		break;
		
		case FIRMWARE:
		putHeaderOnTx(dataTx, FIRMWARE, 12);
		putStrOntx(dataTx, firmware);
		putByteOnTx(dataTx, dataTx->chk);
		break;
		
		case GETDISTANCE:
		putHeaderOnTx(dataTx, GETDISTANCE, 5);
		myWord.ui32 = distancia * 58;
		putByteOnTx(dataTx, myWord.ui8[0] );
		putByteOnTx(dataTx, myWord.ui8[1] );
		putByteOnTx(dataTx, myWord.ui8[2] );
		putByteOnTx(dataTx, myWord.ui8[3] );
		putByteOnTx(dataTx, dataTx->chk);
		break;
		
		case SERVOANGLE:
		putHeaderOnTx(dataTx, SERVOANGLE, 2);
		putByteOnTx(dataTx, ACK);
		putByteOnTx(dataTx, dataTx->chk);

		angleSource = getByteFromRx(dataRx, 1, 0);
		servo_setAngle(servoA, angleSource);  
		break;
		
		case ANALOGSENSORS:
		putHeaderOnTx(dataTx, ANALOGSENSORS, 7);  // 6 bytes de sensores + 1 checksum

		// Sensor IZQUIERDO
		myWord.ui16[0] = (uint16_t)ir_state(ir1);  // ? podés cambiar ir1 por el que corresponda a izquierda
		putByteOnTx(dataTx, myWord.ui8[0]);
		putByteOnTx(dataTx, myWord.ui8[1]);

		// Sensor CENTRAL
		myWord.ui16[0] = (uint16_t)ir_state(ir1);  // ? cambia por el ID correspondiente
		putByteOnTx(dataTx, myWord.ui8[0]);
		putByteOnTx(dataTx, myWord.ui8[1]);

		// Sensor DERECHO
		myWord.ui16[0] = (uint16_t)ir_state(ir1);  // ? cambia por el ID correspondiente
		putByteOnTx(dataTx, myWord.ui8[0]);
		putByteOnTx(dataTx, myWord.ui8[1]);

		putByteOnTx(dataTx, dataTx->chk); // Checksum final
		break;
		
		case BOXCATEGORY:
		putHeaderOnTx(dataTx, BOXCATEGORY, 2);  // 1 byte de payload + checksum
		putByteOnTx(dataTx, (uint8_t)cinta.tipoCaja);  // Envía el enum convertido a uint8_t
		putByteOnTx(dataTx, dataTx->chk);
		break;
		
		case GETALLBOXES:
		putHeaderOnTx(dataTx, GETALLBOXES, boxFila.count + 1);  // +1 por el ID		
		for (uint8_t i = 0; i < boxFila.count; i++) {			// Enviar todos los elementos del buffer
			uint8_t index = (boxFila.head + i) % MAX_BOXES;
			putByteOnTx(dataTx, (uint8_t)boxFila.buffer[index]);
		}
		putByteOnTx(dataTx, dataTx->chk);  // Checksum final
		break;

				
		case SENDALLSENSORS:
			flag1.bit.sendAllSensorsFlag = 1;
		break;
		
		case STOPALLSENSORS:
			flag1.bit.sendAllSensorsFlag = 0;
		break;
		default:
		putHeaderOnTx(dataTx, (_eCmd)dataRx->buff[dataRx->indexData], 2);
		putByteOnTx(dataTx, UNKNOWN);
		putByteOnTx(dataTx, dataTx->chk);
		break;
	}
}

// Función para inicializar el USART
void USART_Init(unsigned int ubrr) {
	UCSR0A |= (1 << U2X0);  // ?? Habilitar doble velocidad sin borrar otros bits
	UBRR0H = (unsigned char)(ubrr >> 8);
	UBRR0L = (unsigned char)ubrr;
	UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
	UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
}

// Función que retorna el tiempo actual en milisegundos
uint32_t millis() {
	uint32_t millis_copy;
	cli();  // Deshabilita interrupciones para evitar problemas de lectura
	millis_copy = timer_millis;
	sei();  // Habilita interrupciones nuevamente
	return millis_copy;
}

// Función para inicializar un delay no bloqueante
void delayConfig(_delay_t *delay, uint16_t interval) {
	delay->startTime = millis();
	delay->interval = interval;
	delay->isRunning = 1;
}

// Función para verificar si el tiempo ha transcurrido
uint8_t delayRead(_delay_t *delay) {
	uint8_t timeReach = 0;
	if (!delay->isRunning) {
		delay->isRunning = 1;
		delay->startTime = millis();
		} else {
		if ((millis() - delay->startTime) >= delay->interval) {
			timeReach = 1;
			delay->startTime = millis(); // Reiniciar internamente
		}
	}
	return timeReach;
}

// Función para modificar el delay y resetearlo
void delayWrite(_delay_t *delay, uint16_t interval) {
	delay->interval = interval;
	delay->startTime = millis();
	delay->isRunning = 1;
}

void delayStop(_delay_t *delay) {
	delay->isRunning = 0;
}

void sendDirect(const char *msg) {
	while (*msg) {
		while (!(UCSR0A & (1 << UDRE0)));
		UDR0 = *msg++;
	}
}

//////////////////////////// FUNCIONES BOX ///////////////////////////
// Inicializa la estructura de la cola vacía
void BoxFila_init(BoxFila *f) {
	f->head = 0;   // Índice de lectura (posición del primer elemento)
	f->tail = 0;   // Índice de escritura (posición libre para el próximo elemento)
	f->count = 0;  // Cantidad actual de elementos en la cola
}

// Devuelve 1 (true) si la cola está vacía, 0 (false) si tiene elementos
uint8_t BoxFila_isEmpty(BoxFila *f) {
	return f->count == 0;
}

// Devuelve 1 (true) si la cola está llena, 0 (false) si hay espacio
uint8_t BoxFila_isFull(BoxFila *f) {
	return f->count == MAX_BOXES;
}

// Agrega un nuevo tipo de caja al final de la cola
uint8_t BoxFila_push(BoxFila *f, BoxCategory category) {
	if (f->count == MAX_BOXES) {
		// Si está lleno, sobrescribimos el más viejo (head) y lo desplazamos
		f->buffer[f->tail] = category;
		f->tail = (f->tail + 1) % MAX_BOXES;
		f->head = (f->head + 1) % MAX_BOXES;  // Mueve el head también
		// f->count no cambia, sigue siendo MAX_BOXES
		} else {
		f->buffer[f->tail] = category;
		f->tail = (f->tail + 1) % MAX_BOXES;
		f->count++;
	}
	return 1;
}


// Elimina y devuelve el primer elemento de la cola (FIFO)
BoxCategory BoxFila_popAll() {
	if (BoxFila_isEmpty(&boxFila)) return CATEGORY_NONE;

	uint8_t index = boxFila.head;
	BoxCategory val = boxFila.buffer[index];
	boxFila.head = (boxFila.head + 1) % MAX_BOXES;
	boxFila.count--;

	// Resetear flags de sectores
	sectorA_flags[index] = 0;
	sectorB_flags[index] = 0;
	sectorC_flags[index] = 0;

	return val;
}


//////////////////////////// FUNCIONES BOX ///////////////////////////


void enviarBoxFila() {
	putHeaderOnTx(&dataTx, GETALLBOXES, boxFila.count + 1);
	for (uint8_t i = 0; i < boxFila.count; i++) {
		uint8_t index = (boxFila.head + i) % MAX_BOXES;
		putByteOnTx(&dataTx, (uint8_t)boxFila.buffer[index]);
	}
	putByteOnTx(&dataTx, dataTx.chk);
}

void enviarCategoria(BoxCategory tipo) {
	putHeaderOnTx(&dataTx, BOXCATEGORY, 2);
	putByteOnTx(&dataTx, (uint8_t)tipo);
	putByteOnTx(&dataTx, dataTx.chk);
}

void enviarCantidadCajasPateadas(void) {
	putHeaderOnTx(&dataTx, CAJASPATEADAS, 2);
	putByteOnTx(&dataTx, contadorCajasPateadas);
	putByteOnTx(&dataTx, dataTx.chk);
}

void manejarClasificacionCaja() {
	if (!cinta.esperandoCaja || !ir_fall(ir0)) return;

	uint32_t distancia_cm = distancia;
	BoxCategory tipo = CATEGORY_NONE;

	if (distancia_cm > ALTURAMINIMA_A && distancia_cm <= ALTURAMAXIMA_A) tipo = CATEGORY_A;
	else if (distancia_cm > ALTURAMINIMA_B && distancia_cm <= ALTURAMAXIMA_B) tipo = CATEGORY_B;
	else if (distancia_cm > ALTURAMINIMA_C && distancia_cm <= ALTURAMAXIMA_C) tipo = CATEGORY_C;

	if (tipo != CATEGORY_NONE) {
		BoxFila_push(&boxFila, tipo);
		
		uint8_t index = boxFila.tail == 0 ? MAX_BOXES - 1 : boxFila.tail - 1;
		sectorA_flags[index] = (tipo == CATEGORY_A) ? 1 : 0;
		sectorB_flags[index] = (tipo == CATEGORY_B) ? 1 : 0;
		sectorC_flags[index] = (tipo == CATEGORY_C) ? 1 : 0;

		cinta.cajaClasificada = 1; // MARCAR QUE HAY UNA CAJA CLASIFICADA
		enviarBoxFila();

		// ? IMPORTANTE: Resetear estados de sensores e índices de flancos
		cinta.cajaEnIR1 = 0;
		cinta.cajaEnIR2 = 0;
		cinta.cajaEnIR3 = 0;
		cinta.ir1_flancoIndex = 0;
		cinta.ir2_flancoIndex = 0;
		cinta.ir3_flancoIndex = 0;
		clasificacionListaParaPateo = 1;
	}

	enviarCategoria(tipo);
	cinta.esperandoCaja = 0;
}

void verificarEsperarNuevaCaja() {
	if (!cinta.esperandoCaja && ir_rise(ir0)) {
		cinta.esperandoCaja = 1;
	}
}

///////////////////////////// SECTOR A ///////////////////////////////////////////////////////////

void verificarPateoCajaTipoA() {
	if (!clasificacionListaParaPateo) return;
	if (BoxFila_isEmpty(&boxFila) || flag2.bit.servoAPateo || flag2.bit.servoAReturning) return;

	if (!cinta.cajaEnIR1 && ir_fall(ir1)) {
		cinta.cajaEnIR1 = 1;
	}

	if (cinta.cajaEnIR1 && ir_rise(ir1)) {
		cinta.cajaEnIR1 = 0;

		// Revisar solo el índice actual (flancoIndex), sin buscar el próximo 1
		uint8_t index = (boxFila.head + cinta.ir1_flancoIndex) % MAX_BOXES;

		if (sectorA_flags[index]) {
			// Si corresponde patear, ejecutar
			servo_setAngle(servoA, 0);
			flag2.bit.servoAPateo = 1;
			delayWrite(&timeoutServoA, 150);
			delayWrite(&ir1Timeout, 500);

			putHeaderOnTx(&dataTx, SERVOAPATEO, 2);
			putByteOnTx(&dataTx, ACK);
			putByteOnTx(&dataTx, dataTx.chk);

			enviarBoxFila();
			contadorCajasPateadas++;
			enviarCantidadCajasPateadas();

			cinta.indiceCajaEnIR1 = index;
			clasificacionListaParaPateo = 0; // ?? se habilita solo 1 pateo por caja clasificada
		}

		// Siempre se incrementa el índice de flancos al detectar flanco de subida
		cinta.ir1_flancoIndex++;
	}
}

void verificarRetornoServoA() {
	if (flag2.bit.servoAPateo && delayRead(&timeoutServoA)) {
		servo_setAngle(servoA, 90);
		flag2.bit.servoAPateo = 0;
		flag2.bit.servoAReturning = 1;
	}
}

void verificarSalidaCajaTipoA() {
	if (flag2.bit.servoAReturning && (ir_rise(ir1) || delayRead(&ir1Timeout))) {
		flag2.bit.servoAReturning = 0;
		delayStop(&timeoutServoA);

		// ?? Eliminar la caja que se pateó
		uint8_t idx = cinta.indiceCajaEnIR1;
		boxFila.buffer[idx] = CATEGORY_NONE;
		sectorA_flags[idx] = 0;
		sectorB_flags[idx] = 0;
		sectorC_flags[idx] = 0;


		cinta.cajaEnIR1 = 0;

		// ? Ajustar flanco index y avanzar head si corresponde
		if (cinta.ir1_flancoIndex > 0) cinta.ir1_flancoIndex--;
		boxFila.head = (boxFila.head + 1) % MAX_BOXES;
		boxFila.count--;
	}
}



/////////////////////////////// SECTOR B //////////////////////////////////////////////////////////

void verificarPateoCajaTipoB() {
	if (!clasificacionListaParaPateo) return;
	if (BoxFila_isEmpty(&boxFila) || flag2.bit.servoBPateo || flag2.bit.servoBReturning) return;

	if (!cinta.cajaEnIR2 && ir_fall(ir2)) {
		cinta.cajaEnIR2 = 1;
	}

	if (cinta.cajaEnIR2 && ir_rise(ir2)) {
		cinta.cajaEnIR2 = 0;

		uint8_t index = (boxFila.head + cinta.ir2_flancoIndex) % MAX_BOXES;

		if (sectorB_flags[index]) {
			servo_setAngle(servoB, 0);
			flag2.bit.servoBPateo = 1;
			delayWrite(&timeoutServoB, 150);
			delayWrite(&ir2Timeout, 500);

			putHeaderOnTx(&dataTx, SERVOBPATEO, 2);
			putByteOnTx(&dataTx, ACK);
			putByteOnTx(&dataTx, dataTx.chk);

			enviarBoxFila();
			contadorCajasPateadas++;
			enviarCantidadCajasPateadas();

			cinta.indiceCajaEnIR2 = index;
			clasificacionListaParaPateo = 0; // ?? se habilita solo 1 pateo por caja clasificada
		}

		cinta.ir2_flancoIndex++;
	}
}

void verificarRetornoServoB() {
	if (flag2.bit.servoBPateo && delayRead(&timeoutServoB)) {
		servo_setAngle(servoB, 90);
		flag2.bit.servoBPateo = 0;
		flag2.bit.servoBReturning = 1;
	}
}

void verificarSalidaCajaTipoB() {
	if (flag2.bit.servoBReturning && (ir_rise(ir2) || delayRead(&ir2Timeout))) {
		flag2.bit.servoBReturning = 0;
		delayStop(&timeoutServoB);

		uint8_t idx = cinta.indiceCajaEnIR2;
		boxFila.buffer[idx] = CATEGORY_NONE;
		sectorA_flags[idx] = 0;
		sectorB_flags[idx] = 0;
		sectorC_flags[idx] = 0;


		cinta.cajaEnIR2 = 0;

		// ? Ajustar flanco index y avanzar head si corresponde
		if (cinta.ir2_flancoIndex > 0) cinta.ir2_flancoIndex--;
		boxFila.head = (boxFila.head + 1) % MAX_BOXES;
		boxFila.count--;
	}
}

//////////////////////////////////// SECTOR C /////////////////////////////////////////////////////

void verificarPateoCajaTipoC() {
	if (!clasificacionListaParaPateo) return;
	if (BoxFila_isEmpty(&boxFila) || flag2.bit.servoCPateo || flag2.bit.servoCReturning) return;

	if (!cinta.cajaEnIR3 && ir_fall(ir3)) {
		cinta.cajaEnIR3 = 1;
	}

	if (cinta.cajaEnIR3 && ir_rise(ir3)) {
		cinta.cajaEnIR3 = 0;

		uint8_t index = (boxFila.head + cinta.ir3_flancoIndex) % MAX_BOXES;

		if (sectorC_flags[index]) {
			servo_setAngle(servoC, 0);
			flag2.bit.servoCPateo = 1;
			delayWrite(&timeoutServoC, 150);
			delayWrite(&ir3Timeout, 500);

			putHeaderOnTx(&dataTx, SERVOCPATEO, 2);
			putByteOnTx(&dataTx, ACK);
			putByteOnTx(&dataTx, dataTx.chk);

			enviarBoxFila();
			contadorCajasPateadas++;
			enviarCantidadCajasPateadas();
			
			cinta.indiceCajaEnIR3 = index;
			clasificacionListaParaPateo = 0; // ?? se habilita solo 1 pateo por caja clasificada
		}

		cinta.ir3_flancoIndex++;
	}
}

void verificarRetornoServoC() {
	if (flag2.bit.servoCPateo && delayRead(&timeoutServoC)) {
		servo_setAngle(servoC, 90);
		flag2.bit.servoCPateo = 0;
		flag2.bit.servoCReturning = 1;
	}
}

void verificarSalidaCajaTipoC() {
	if (flag2.bit.servoCReturning && (ir_rise(ir3) || delayRead(&ir3Timeout))) {
		flag2.bit.servoCReturning = 0;
		delayStop(&timeoutServoC);

		uint8_t idx = cinta.indiceCajaEnIR3;
		boxFila.buffer[idx] = CATEGORY_NONE;
		sectorA_flags[idx] = 0;
		sectorB_flags[idx] = 0;
		sectorC_flags[idx] = 0;

		cinta.cajaEnIR3 = 0;

		// ? Ajustar flanco index y avanzar head si corresponde
		if (cinta.ir3_flancoIndex > 0) cinta.ir3_flancoIndex--;
		boxFila.head = (boxFila.head + 1) % MAX_BOXES;
		boxFila.count--;
	}
}

void cintaTask() {
	manejarClasificacionCaja();
	verificarEsperarNuevaCaja();

	verificarPateoCajaTipoA();
	verificarRetornoServoA();
	verificarSalidaCajaTipoA();

	verificarPateoCajaTipoB();
	verificarRetornoServoB();
	verificarSalidaCajaTipoB();

	verificarPateoCajaTipoC();
	verificarRetornoServoC();
	verificarSalidaCajaTipoC();
}

/* END Function prototypes user code ------------------------------------------*/


int main() {
	/* Local variables -----------------------------------------------------------*/

	/* END Local variables -------------------------------------------------------*/

	/* User code Init ------------------------------------------------------------*/	
	DDRB |= (1 << LED) | (1 << TRIG);  // Configurar LED y TRIG como salida
	DDRB &= ~(1 << PB0); // Configurar PB0 (ECHO) como entrada
	
	timer0_init();
	Timer2_Init();  // Usado para millis()
	servo_init();  // Llama a la función para configurar Timer0 y los pines de los servos	
	
		
	// Valores de los IR, 1 sin objeto adelante - 0 con objeto adelante
	ir0 = ir_add(&PIND, PD2); // Infrarojo de ENTRADA (al lado del HCSR04)
	ir1 = ir_add(&PIND, PD3);	// Infrarrojo que coincide con el servoA
	ir2 = ir_add(&PIND, PD4);	// Infrarrojo que coincide con el servoB
	ir3 = ir_add(&PIND, PD5);	// Infrarrojo que coincide con el servoB
	
	delayConfig(&generalTime, 10);  // Delay de 10ms
	delayConfig(&oneSecond, 1000);  // Delay de 1000ms (1s)
	delayConfig(&allSensorsTime, 500);  // Delay to take all data in one single send
	
	dataRx.buff = (uint8_t*)buffRx;
	dataRx.mask = RXBUFSIZE - 1;
	dataRx.indexW = 0;
	dataRx.indexR = 0;
	dataRx.header = HEADER_U;
	dataRx.isComannd = 0;

	dataTx.buff = buffTx;
	dataTx.mask = TXBUFSIZE - 1;
	dataTx.indexW = 0;
	dataTx.indexR = 0;
	
	HCSR04_Init();
	USART_Init(MYUBRR);
	
	BoxFila_init(&boxFila);
	
	cinta.esperandoCaja = 1;
	flag2.byte = 0;
	cinta.cajaEnIR1 = 0;
	cinta.cajaEnIR2 = 0;
	cinta.cajaEnIR3 = 0;  
	cinta.ir1_flancoIndex = 0;
	cinta.ir2_flancoIndex = 0;
	cinta.ir3_flancoIndex = 0;
	contadorCajasPateadas = 0;
	clasificacionListaParaPateo = 0;

	memset(sectorA_flags, 0, MAX_BOXES);
	memset(sectorB_flags, 0, MAX_BOXES);
	memset(sectorC_flags, 0, MAX_BOXES);
	
	servoA = servo_add(&PORTD, (1 << SERVO_PIN));
	servoB = servo_add(&PORTB, (1 << SERVO2_PIN));
	servoC = servo_add(&PORTB, (1 << SERVO3_PIN));
	
	
	servo_setAngle(servoA, 90);
	servo_setAngle(servoB, 90);
	servo_setAngle(servoC, 90);
	sei();  // Habilita interrupciones globales
	
	/* END User code Init --------------------------------------------------------*/
	
	while (1) {
		/* User Code loop ------------------------------------------------------------*/
		serialTask((_sRx *)&dataRx,&dataTx, 0);
		
		if (flag1.bit.sendAllSensorsFlag && delayRead(&allSensorsTime)) {
			putHeaderOnTx(&dataTx, GETDISTANCE, 5);
			myWord.ui32 = distancia * 58;
			putByteOnTx(&dataTx, myWord.ui8[0]);
			putByteOnTx(&dataTx, myWord.ui8[1]);
			putByteOnTx(&dataTx, myWord.ui8[2]);
			putByteOnTx(&dataTx, myWord.ui8[3]);
			putByteOnTx(&dataTx, dataTx.chk);
		}
		
		HCSR04_Task();
		if (HCSR04_isNewMeasurement()) {	// Hay una nueva medicion
			uint32_t tiempo_us = HCSR04_getDistance();
			distancia = tiempo_us / 58;
		}
		
		if(delayRead(&generalTime)) {	// Funciones llamadas cada 10ms
			ir_update_all();
			cintaTask();	
		}					
		
		/* END User Code loop --------------------------------------------------------*/
	}

	return 0;
}