/*
 * GPIO_wrapper.c
 *
 *  Created on: 17 abr. 2018
 *      Author: SINAPSE-FLG
 */


/*****************************************************************************
 *  Include section
 *****************************************************************************/
#include "GPIO_wrapper.h"
#include "stm32f2xx_hal.h"
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "Math.h"

/*****************************************************************************
 * Function name    : EMU_GPIO_config
 * 	  @brief		: Configure a specific GPIO signal
 *
 *    @param1       : SIGNAL: pointer to a EMU_GPIO_Signal structure that contains
 *         		  	  the HW-Parse information for a specified signal
 *    @param2       : PXY: microcontroller Pin identifier. X: Port Y: Pin.
 *    				  This parameter can be one of the EMU_GPIO_Pins enum values
 *    @param3       : MODE: GPIO Configuration Mode.
 *    				  This parameter can be one of the EMU_GPIO_Mode enum values
 *    @param4       : PULL: GPIO Pull-Up or Pull-Down Activation.
 *    				  This parameter can be one of the EMU_GPIO_Pull enum values
 * 	  @retval       : bool, true if successful
 *
 * Notes            :
 *****************************************************************************/
bool EMU_GPIO_config(EMU_GPIO_Signal *SIGNAL, EMU_GPIO_Pins PXY, EMU_GPIO_Mode MODE, EMU_GPIO_Pull PULL)
{
	GPIO_InitTypeDef GPIO_InitStruct;			//GPIO Init structure
	uint8_t mod = (PXY%16);;					//"Mask" value to select a specific GPIO_PIN_X
	uint8_t div = (PXY / 16);					//value to select the specific GPIO Port

	/*Fill SIGNAL struct with the specified PORT*/
	switch (div){

		case 0:
			SIGNAL->PORT=GPIOA;
			break;
		case 1:
			SIGNAL->PORT=GPIOB;
			break;
		case 2:
			SIGNAL->PORT=GPIOC;
			break;
		case 3:
			SIGNAL->PORT=GPIOD;
			break;
	}

	/*Fill SIGNAL struct with the specified PIN*/
	SIGNAL->PIN = (GPIO_PIN_0)<<mod;
	SIGNAL->numberPIN=mod;
	/*Specifies the GPIO pin to be configured.*/
	GPIO_InitStruct.Pin = (GPIO_PIN_0)<<mod;

	/*Specifies the operating mode for the selected pins.*/
	switch (MODE)
	{
		case INPUT:
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			break;
		case OUTPUT:
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			/*Specifies the speed for the selected pins, only in OUTPUT mode*/
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
			break;
		case OPENDRAIN:
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
			/*Specifies the speed for the selected pins, only in OUTPUT mode*/
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
			break;
		case IT_RISING:
			GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
			break;
		case IT_FALLING:
			GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
			break;
		case IT_RISING_FALLING:
			GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
			break;
		case EVT_RISING:
			GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING ;
			break;
		case EVT_FALLING:
			GPIO_InitStruct.Mode = GPIO_MODE_EVT_FALLING;
			break;
		case EVT_RISING_FALLING:
			GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING_FALLING;
			break;
	}
	/*Specifies the Pull-up or Pull-Down activation for the selected pins.*/
	switch (PULL)
	{
		case NONE:
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			break;
		case DOWN:
			GPIO_InitStruct.Pull = GPIO_PULLDOWN;
			break;
		case UP:
			GPIO_InitStruct.Pull = GPIO_PULLUP;
			break;
	}

	/*Initialize a specific GPIO signal*/
	if(SIGNAL->PORT==GPIOA){
		__HAL_RCC_GPIOA_CLK_ENABLE();				/* GPIO Ports Clock Enable */
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);		/* Initialize the GPIOA peripheral according to the specified parameters in the GPIO_Init.*/
	}
	if(SIGNAL->PORT==GPIOB){
		__HAL_RCC_GPIOB_CLK_ENABLE();				/* GPIO Ports Clock Enable */
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);		/*Initialize the GPIOB peripheral according to the specified parameters in the GPIO_Init.*/
	}
	if(SIGNAL->PORT==GPIOC){
		__HAL_RCC_GPIOC_CLK_ENABLE();				/* GPIO Ports Clock Enable */
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);		/*Initialize the GPIOC peripheral according to the specified parameters in the GPIO_Init.*/
	}
	if(SIGNAL->PORT==GPIOD){
		__HAL_RCC_GPIOD_CLK_ENABLE();				/* GPIO Ports Clock Enable */
		HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);		/*Initialize the GPIOD peripheral according to the specified parameters in the GPIO_Init.*/
	}

	return true;
}


/*****************************************************************************
 * Function name    : EMU_GPIO_SetMode
 * 	  @brief		: Change a specific GPIO Pin mode
 *
 *    @param1       : SIGNAL: pointer to a EMU_GPIO_Signal structure that contains
 *         		  	  the HW-Parse information for a specified signal
 *    @param2       : MODE: GPIO Configuration Mode.
 *    				  This parameter can be one of the EMU_GPIO_Mode enum values
 * 	  @retval       : bool, true if successful
 *
 * 	  Notes         : Actual version only implements INPUT, OUTPUT and OPENDRAIN modes
 *****************************************************************************/
bool EMU_GPIO_SetMode(EMU_GPIO_Signal *SIGNAL, EMU_GPIO_Mode MODE)
{

	switch(MODE){

		case INPUT:
			SIGNAL->PORT->MODER &=~(1<<2*(SIGNAL->numberPIN));
			break;
		case OUTPUT:
			SIGNAL->PORT->MODER|=(1<<2*(SIGNAL->numberPIN));
			break;
		case OPENDRAIN:
			SIGNAL->PORT->MODER|=(1<<2*(SIGNAL->numberPIN));
			SIGNAL->PORT->OTYPER|=(1<<(SIGNAL->numberPIN));

	}

	return true;
}

/*****************************************************************************
 * Function name    : EMU_GPIO_Read_Write
 * 	  @brief		: Function that Read or Write 1/0 in a specific GPIO
 *
 *    @param1       : ID_SENSOR: pointer to a SensorType structure that contains
 *         		  	  the signals definition for a specific digital Sensor.
 *    @param2       : Stream_CS: pointer to a string to emulate CS signal
 *    @param3		: Stream_SDI: pointer to a string to emulate SDI signal
 *    @param4		: Stream_SDO: pointer to a string to emulate SDO signal
 *    @param5		: Stream_SCK: pointer to a string to emulate SCK signal
 *    @param6		: Signals: pointer to a SignalsValue structure where the read values
 *    				  are stored.
 *
 * 	  @retval       : bool, true if successful
 *
 * 	  Notes         : Actual version only implements INPUT, OUTPUT and OPENDRAIN modes
 *****************************************************************************/
bool EMU_GPIO_Read_Write(SensorType *ID_SENSOR, char *Stream_CS, char *Stream_SDI,  char* Stream_SDO,
							 char *Stream_SCK, SignalsValue *Signals)
{
	uint8_t tam_CS = strlen((const char*)Stream_CS);
	uint8_t tam_SDI = strlen((const char*)Stream_SDI);
	uint8_t tam_SDO = strlen((const char*)Stream_SDO);
	uint8_t tam_SCK = strlen((const char*)Stream_SCK);
	uint8_t tam_Stream = 0;
	uint16_t i = 0, cnt = 0, cnt1 =0, cnt2 = 0, cnt3 = 0, cnt4 = 0;
	uint32_t result=0;

	//Check if Stream signals have the same size
	if ((tam_CS == tam_SDI) && (tam_SDO == tam_SCK))
	{
		tam_Stream = tam_SCK;

		for (i = 0; i < tam_Stream; i++)
		{

			if(Stream_SCK[i] == '1')
			{
				//Check GPIO port output type and reconfigure GPIO mode (redundancy)
				if((ID_SENSOR->SCK_signal.PORT->OTYPER & ID_SENSOR->SCK_signal.PIN) == ID_SENSOR->SCK_signal.PIN)
					EMU_GPIO_SetMode(&(ID_SENSOR->SCK_signal), OPENDRAIN);
				else
					EMU_GPIO_SetMode(&(ID_SENSOR->SCK_signal), OUTPUT);

				ID_SENSOR->SCK_signal.PORT->BSRR = (1 << ID_SENSOR->SCK_signal.numberPIN); //GPIO port bit set

			}

			else if(Stream_SCK[i] == '0')
			{
				//Check GPIO port output type and reconfigure GPIO mode (redundancy)
				if((ID_SENSOR->SCK_signal.PORT->OTYPER & ID_SENSOR->SCK_signal.PIN) == ID_SENSOR->SCK_signal.PIN)
					EMU_GPIO_SetMode(&(ID_SENSOR->SCK_signal), OPENDRAIN);
				else
					EMU_GPIO_SetMode(&(ID_SENSOR->SCK_signal), OUTPUT);

				ID_SENSOR->SCK_signal.PORT->BSRR = (GPIO_BSRR_BR0 << (ID_SENSOR->SCK_signal.numberPIN)); //GPIO port bit reset

			}

			else if(Stream_SCK[i] == 'R')
			{
				//Change GPIO to INPUT mode
				EMU_GPIO_SetMode(&(ID_SENSOR->SCK_signal), INPUT);

				//Check GPIO port input data register
				result = ID_SENSOR->SCK_signal.PORT->IDR & ID_SENSOR->SCK_signal.PIN;
				//Save the read value in SignalsValue structure
				if(result == 0)	Signals->SCK_Readvalue[cnt4] = 0;
				else Signals->SCK_Readvalue[cnt4] = 1;
				cnt4++;
			}

			if(Stream_CS[i] == '1')
			{
				//Check GPIO port output type and reconfigure GPIO mode (redundancy)
				if((ID_SENSOR->CS_signal.PORT->OTYPER & ID_SENSOR->CS_signal.PIN) == ID_SENSOR->CS_signal.PIN)
					EMU_GPIO_SetMode(&(ID_SENSOR->CS_signal), OPENDRAIN);
				else
					EMU_GPIO_SetMode(&(ID_SENSOR->CS_signal), OUTPUT);

				//ID_SENSOR->CS_signal.PORT->BSRR = ID_SENSOR->CS_signal.PIN;	//GPIO port bit set
				ID_SENSOR->CS_signal.PORT->BSRR = (1 << ID_SENSOR->CS_signal.numberPIN);
			}

			else if(Stream_CS[i] == '0')
			{
				//Check GPIO port output type and reconfigure GPIO mode (redundancy)
				if((ID_SENSOR->CS_signal.PORT->OTYPER & ID_SENSOR->CS_signal.PIN) == ID_SENSOR->CS_signal.PIN)
					EMU_GPIO_SetMode(&(ID_SENSOR->CS_signal), OPENDRAIN);
				else
					EMU_GPIO_SetMode(&(ID_SENSOR->CS_signal), OUTPUT);

				ID_SENSOR->CS_signal.PORT->BSRR = (GPIO_BSRR_BR0 << (ID_SENSOR->CS_signal.numberPIN)); //GPIO port bit reset
			}

			else if(Stream_CS[i] == 'R')
			{
				//Change GPIO to INPUT mode
				EMU_GPIO_SetMode(&(ID_SENSOR->CS_signal), INPUT);

				//Check GPIO port input data register
				result = ID_SENSOR->CS_signal.PORT->IDR & ID_SENSOR->CS_signal.PIN;
				//Save the read value in SignalsValue structure
				if(result == 0)	Signals->CS_Readvalue[cnt1] = 0;
				else Signals->CS_Readvalue[cnt1] = 1;
				cnt1++;
			}


			if(Stream_SDI[i] == '1')
			{
				//Check GPIO port output type and reconfigure GPIO mode (redundancy)
				if((ID_SENSOR->SDI_signal.PORT->OTYPER & ID_SENSOR->SDI_signal.PIN) == ID_SENSOR->SDI_signal.PIN)
					EMU_GPIO_SetMode(&(ID_SENSOR->SDI_signal), OPENDRAIN);
				else
					EMU_GPIO_SetMode(&(ID_SENSOR->SDI_signal), OUTPUT);

				ID_SENSOR->SDI_signal.PORT->BSRR = (1 << ID_SENSOR->SDI_signal.numberPIN);
			}

			else if(Stream_SDI[i] == '0')
			{
				//Check GPIO port output type and reconfigure GPIO mode (redundancy)
				if((ID_SENSOR->SDI_signal.PORT->OTYPER & ID_SENSOR->SDI_signal.PIN) == ID_SENSOR->SDI_signal.PIN)
					EMU_GPIO_SetMode(&(ID_SENSOR->SDI_signal), OPENDRAIN);
				else
					EMU_GPIO_SetMode(&(ID_SENSOR->SDI_signal), OUTPUT);

				//ID_SENSOR->SDI_signal.PORT->BRR = ID_SENSOR->SDI_signal.PIN;	//GPIO port bit reset
				ID_SENSOR->SDI_signal.PORT->BSRR = (GPIO_BSRR_BR0 << (ID_SENSOR->SDI_signal.numberPIN)); //GPIO port bit reset
			}
			else if(Stream_SDI[i] == 'R')
			{
				//Change GPIO to INPUT mode
				EMU_GPIO_SetMode(&(ID_SENSOR->SDI_signal), INPUT);

				//Check GPIO port input data register
				result = ID_SENSOR->SDI_signal.PORT->IDR & ID_SENSOR->SDI_signal.PIN;
				//Save the read value in SignalsValue structure
				if(result == 0)	Signals->SDI_Readvalue[cnt2] = 0;
				else Signals->SDI_Readvalue[cnt2] = 1;
				cnt2++;
			}

			if(Stream_SDO[i] == '1')
			{
				//Check GPIO port output type and reconfigure GPIO mode (redundancy)
				if((ID_SENSOR->SDO_signal.PORT->OTYPER & ID_SENSOR->SDO_signal.PIN) == ID_SENSOR->SDO_signal.PIN)
					EMU_GPIO_SetMode(&(ID_SENSOR->SDO_signal), OPENDRAIN);
				else
					EMU_GPIO_SetMode(&(ID_SENSOR->SDO_signal), OUTPUT);

				//ID_SENSOR->SDO_signal.PORT->BSRR = ID_SENSOR->SDO_signal.PIN;	//GPIO port bit set
				ID_SENSOR->SDO_signal.PORT->BSRR = (1 << ID_SENSOR->SDO_signal.numberPIN);
			}

			else if(Stream_SDO[i] == '0')
			{
				//Check GPIO port output type and reconfigure GPIO mode (redundancy)
				if((ID_SENSOR->SDO_signal.PORT->OTYPER & ID_SENSOR->SDO_signal.PIN) == ID_SENSOR->SDO_signal.PIN)
					EMU_GPIO_SetMode(&(ID_SENSOR->SDO_signal), OPENDRAIN);
				else
					EMU_GPIO_SetMode(&(ID_SENSOR->SDO_signal), OUTPUT);

				//ID_SENSOR->SDO_signal.PORT->BRR = ID_SENSOR->SDO_signal.PIN;	//GPIO port bit reset
				ID_SENSOR->SDO_signal.PORT->BSRR = (GPIO_BSRR_BR0 << (ID_SENSOR->SDO_signal.numberPIN)); //GPIO port bit reset
			}

			else if(Stream_SDO[i] == 'R')
			{
				//Change GPIO to INPUT mode
				EMU_GPIO_SetMode(&(ID_SENSOR->SDO_signal), INPUT);

				//Check GPIO port input data register
				result = ID_SENSOR->SDO_signal.PORT->IDR & ID_SENSOR->SDO_signal.PIN;
				//Save the read value in SignalsValue structure
				if(result == 0)	Signals->SDO_Readvalue[cnt3] = 0;
				else Signals->SDO_Readvalue[cnt3] = 1;
				cnt3++;
			}

			//Delay to stabilize the signal
			__asm("nop");
			__asm("nop");
			__asm("nop");
			__asm("nop");
			__asm("nop");

			for(cnt = 0; cnt < ID_SENSOR->TIMMING_pulse; cnt++){
				__asm("nop");
			}
		}
		return true;
	}
	else
		return false;
}

/*****************************************************************************
 * Function name    : config_Sensor
 * 	  @brief		: Configure a specific digital sensor
 *
 *    @param1       : SENSOR: pointer to a SensorType structure that contains
 *    				  the signals definition for a specific digital sensor
 *    @param2       : Signal1: EMU_GPIO_Signal for CS_Signal
 *    @param3		: Signal2: idem for SCK_Signal
 *    @param4		: Signal3: idem for SDI_Signal
 *    @param5		: Signal4: idem for SDO_Signal
 *    @param6		: WPULSE: pulse width value
 *
 * 	  @retval       : void
 *
 * 	  Notes         :
 *****************************************************************************/
void config_Sensor(SensorType *SENSOR, EMU_GPIO_Signal Signal1, EMU_GPIO_Signal Signal2, EMU_GPIO_Signal Signal3,
		EMU_GPIO_Signal Signal4, uint16_t WPULSE)
{
	SENSOR->CS_signal = Signal1;
	SENSOR->SCK_signal = Signal2;
	SENSOR->SDI_signal = Signal3;
	SENSOR->SDO_signal = Signal4;
	SENSOR->TIMMING_pulse = WPULSE;
}


/*****************************************************************************
 * Function name    : TemperatureMeasure
 * 	  @brief		: Auxiliary function that calculates the temperature from SHT10 sensor
 * 	  				  More details in: https://www.sparkfun.com/datasheets/Sensors/SHT1x_datasheet.pdf
 *
 *    @param1       : array1: bytes array 1/0 with the first byte from sensor data read value
 *    @param2       : array2: bytes array 1/0 with the second byte from sensor data read value
 *
 * 	  @retval       : float Temperature value
 *
 * 	  Notes         : This function is only used for DEBUG purpose
 *****************************************************************************/
float TemperatureMeasure (char *array1, char *array2)
{
	float d2 = 0.01;
	float d1 = -39.7;
	float temperature = 0.0;

	uint8_t byte1 = 0, byte2;

	byte1 = ConvertArraytoInteger (array1, TAM_ARRAY);
	byte2 = ConvertArraytoInteger (array2, TAM_ARRAY);

	uint16_t integervalue = 256*byte1 + byte2;

	temperature = integervalue*d2 + d1;

	return temperature;

}

/*****************************************************************************
 * Function name    : RHMeasure
 * 	  @brief		: Auxiliary function that calculates the relative humidity from SHT10 sensor
 * 	  				  More details in: https://www.sparkfun.com/datasheets/Sensors/SHT1x_datasheet.pdf
 *
 *    @param1       : array1: bytes array 1/0 with the first byte from sensor data read value
 *    @param2       : array2: bytes array 1/0 with the second byte from sensor data read value
 *
 * 	  @retval       : float Temperature value
 *
 * 	  Notes         : This function is only used for DEBUG purpose
 *****************************************************************************/
float RHMeasure (char *array1, char *array2, float TA)
{
	float c1 = -2.0468;
	float c2 = 0.0367;
	float c3 = -1.5955e-6;
	float t1 = 0.01;
	float t2 = 0.00008;
	float RHlinear = 0, RHtrue = 0;

	uint8_t byte1 = 0, byte2;

	byte1 = ConvertArraytoInteger (array1, TAM_ARRAY);
	byte2 = ConvertArraytoInteger (array2, TAM_ARRAY);

	uint16_t integervalue = 256*byte1 + byte2;

	RHlinear = c1 + c2*integervalue + c3* (pow(integervalue, 2));
	RHtrue = (TA-25)*(t1 + t2*integervalue)+RHlinear;

	return RHtrue;

}

/*****************************************************************************
 * Function name    : ConvertArraytoInteger
 * 	  @brief		: Auxiliary function to convert a boolean array to decimal value
 *
 *    @param1       : array1: bytes array 1/0
 *    @param2       : tam: size array
 *
 * 	  @retval       : Returns a decimal value
 *
 * 	  Notes         : This function is only used for DEBUG purpose
 *****************************************************************************/
uint8_t ConvertArraytoInteger(char *array, uint8_t tam)
{

	uint8_t ret = 0;
	uint8_t tmp;

	for (uint8_t i = 0; i < tam; i++) {
		tmp = array[i];
		ret |= tmp << (tam - i - 1);
	}
	return ret;
}

