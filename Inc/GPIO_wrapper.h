/*
 * GPIO_wrapper.h
 *
 *  Created on: 17 abr. 2018
 *      Author: SINAPSE-PACO
 */

#ifndef GPIO_WRAPPER_H_
#define GPIO_WRAPPER_H_

/*****************************************************************************
 *  Include section
 *****************************************************************************/
#include "stm32f2xx_hal.h"
#include "string.h"
#include "stdbool.h"


#define NUMBYTES 1						//Number of bytes to receive from a specific GPIO Input signal
#define TAM_ARRAY NUMBYTES*8			//Array size

/**
  * @brief   EMU_GPIO_Signal structure definition
  */
typedef struct{

	GPIO_TypeDef *PORT;				//Pointer to Peripheral_declaration GPIO Port
	uint16_t PIN;					//GPIO_PIN number with "mask". Ex: GPIO_PIN_2 = ((uint16_t)0x0004)
	uint16_t numberPIN;				//Integer Pin value: 0 to 15
}EMU_GPIO_Signal;

/**
  * @brief  EMU_GPIO_Mode enumeration
  */
typedef enum {
	INPUT = 0, OUTPUT = 1, OPENDRAIN = 2, IT_RISING = 3, IT_FALLING = 4,
	IT_RISING_FALLING = 5, EVT_RISING = 6, EVT_FALLING = 7, EVT_RISING_FALLING = 8
}EMU_GPIO_Mode;

/**
  * @brief  EMU_GPIO_Pull enumeration
  */
typedef enum {
	NONE = -1, DOWN = 0, UP = 1
}EMU_GPIO_Pull;

/**
  * @brief  EMU_GPIO_Pins enumeration
  *
  * @Notes: This enum must be modified according the specific Microcontroller in use
  */
typedef enum {
	PA0 = 0, PA1 = 1, PA2 = 2, PA3 = 3, PA4 = 4, PA5 = 5, PA6 = 6, PA7 = 7, PA8 = 8, PA9 = 9, PA10 = 10, PA11 = 11, PA12 = 12, PA13 = 13, PA14 =14, PA15 = 15,
	PB0 = 16, PB1 = 17, PB2 = 18, PB3 = 19, PB4 = 20, PB5 = 21, PB6 = 22, PB7 = 23, PB8 = 24, PB9 = 25, PB10 = 26, PB11 = 27, PB12 = 28, PB13 = 29, PB14 =30, PB15 = 31,
	PC0 = 32, PC1 = 33, PC2 = 34, PC3 = 35, PC4 = 36, PC5 = 37, PC6 = 38, PC7 = 39, PC8 = 40, PC9 = 41, PC10 = 42, PC11 = 43, PC12 = 44, PC13 = 45, PC14 =46, PC15 = 47,
	PD0 = 48, PD1 = 49, PD2 = 50, PD3 = 51, PD4 = 52, PD5 = 53, PD6 = 54, PD7 = 55, PD8 = 56, PD9 = 57, PD10 = 58, PD11 = 59, PD12 = 60, PD13 = 61, PD14 =62, PD15 = 63,
	PE0 = 64, PE1 = 65, PE2 = 66, PE3 = 67, PE4 = 68, PE5 = 69, PE6 = 70, PE7 = 71, PE8 = 72, PE9 = 73, PE10 = 74, PE11 = 75, PE12 = 76, PE13 = 77, PE14 =78, PE15 = 79

}EMU_GPIO_Pins;

/**
  * @brief  SensorType structure definition
  *
  * @Notes: This structure contains the 4 signals needed to emulate any sensor with digital signals.
  * 		Each signal is a EMU_GPIO_Signal structure. TIMMING_pulse represents the pulse width
  * 		for each signal.
  */
typedef struct{

	EMU_GPIO_Signal CS_signal;
	EMU_GPIO_Signal SCK_signal;
	EMU_GPIO_Signal SDI_signal;
	EMU_GPIO_Signal SDO_signal;
	uint16_t TIMMING_pulse;
}SensorType;

/**
  * @brief  SignalsValue structure definition
  *
  * @Notes: This structure contains four arrays with the value read for each signal.
  * 		TAM_ARRAY could be modified as needed. By default, TAM_ARRAY is 8
  */
typedef struct{

	char CS_Readvalue[TAM_ARRAY];
	char SDI_Readvalue[TAM_ARRAY];
	char SDO_Readvalue[TAM_ARRAY];
	char SCK_Readvalue[TAM_ARRAY];

}SignalsValue;

/* Initialization and definitions functions *****************************/
bool EMU_GPIO_config(EMU_GPIO_Signal *SIGNAL, EMU_GPIO_Pins PXY, EMU_GPIO_Mode MODE, EMU_GPIO_Pull PULL);
bool EMU_GPIO_SetMode(EMU_GPIO_Signal *SIGNAL, EMU_GPIO_Mode MODE);
void EMU_GPIO_SetupSensor(SensorType *SENSOR, EMU_GPIO_Signal Signal1, EMU_GPIO_Signal Signal2, EMU_GPIO_Signal Signal3,
		EMU_GPIO_Signal Signal4, uint16_t WPULSE);

bool EMU_GPIO_Read_Write(SensorType *ID_SENSOR, char *Stream_CS, char *Stream_SDI,char* Stream_SDO,
							char *Stream_SCK, SignalsValue *Signals);

void GPIO_check(EMU_GPIO_Signal signal, char *genericStream, char *ReadValue, uint8_t index);



#endif /* GPIO_WRAPPER_H_ */
