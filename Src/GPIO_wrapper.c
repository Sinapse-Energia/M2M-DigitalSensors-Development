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
 * Notes            : This function makes:
 * 						1) Init the SIGNAL structure that is part of a digital sensor. This structure will be used by other funcitons
 * 						2) Config the GPIO HW in the same way that "MX_GPIO_Init()" function.
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
	uint16_t i = 0, timming_counter = 0, CS_read_counter =0, SDI_read_counter = 0, SDO_read_counter = 0, SCK_read_counter = 0;
	uint32_t result=0;

	//Check if Stream signals have the same size
	if ((tam_CS == tam_SDI) && (tam_CS == tam_SCK) && (tam_CS == tam_SDO))
	{
		tam_Stream = tam_SCK;

		for (i = 0; i < tam_Stream; i++)
		{

			/*if(Stream_SCK[i] == '1')
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

			}*/
			GPIO_check(ID_SENSOR->SCK_signal, Stream_SCK, Signals->SCK_Readvalue, i);
			//THE FOLLOWING CODE IS KEPT BECAUSE A BUG IN "GPIO_check()" FUNCTION.
			if(Stream_SCK[i] == 'R')
			{
				//Change GPIO to INPUT mode
				EMU_GPIO_SetMode(&(ID_SENSOR->SCK_signal), INPUT);

				//Check GPIO port input data register
				result = ID_SENSOR->SCK_signal.PORT->IDR & ID_SENSOR->SCK_signal.PIN;
				//Save the read value in SignalsValue structure
				if(result == 0)	Signals->SCK_Readvalue[SCK_read_counter] = 0;
				else Signals->SCK_Readvalue[SCK_read_counter] = 1;
				SCK_read_counter++;
			}

			/*if(Stream_CS[i] == '1')
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
			}*/
			GPIO_check(ID_SENSOR->CS_signal, Stream_CS, Signals->CS_Readvalue, i);
			//THE FOLLOWING CODE IS KEPT BECAUSE A BUG IN "GPIO_check()" FUNCTION.
			if(Stream_CS[i] == 'R')
			{
				//Change GPIO to INPUT mode
				EMU_GPIO_SetMode(&(ID_SENSOR->CS_signal), INPUT);

				//Check GPIO port input data register
				result = ID_SENSOR->CS_signal.PORT->IDR & ID_SENSOR->CS_signal.PIN;
				//Save the read value in SignalsValue structure
				if(result == 0)	Signals->CS_Readvalue[CS_read_counter] = 0;
				else Signals->CS_Readvalue[CS_read_counter] = 1;
				CS_read_counter++;
			}


			/*if(Stream_SDI[i] == '1')
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
			}*/
			GPIO_check(ID_SENSOR->SDI_signal, Stream_SDI, Signals->SDI_Readvalue, i);
			//THE FOLLOWING CODE IS KEPT BECAUSE A BUG IN "GPIO_check()" FUNCTION.
			if(Stream_SDI[i] == 'R')
			{
				//Change GPIO to INPUT mode
				EMU_GPIO_SetMode(&(ID_SENSOR->SDI_signal), INPUT);

				//Check GPIO port input data register
				result = ID_SENSOR->SDI_signal.PORT->IDR & ID_SENSOR->SDI_signal.PIN;
				//Save the read value in SignalsValue structure
				if(result == 0)	Signals->SDI_Readvalue[SDI_read_counter] = 0;
				else Signals->SDI_Readvalue[SDI_read_counter] = 1;
				SDI_read_counter++;
			}

			/*if(Stream_SDO[i] == '1')
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
			}*/
			GPIO_check(ID_SENSOR->SDO_signal, Stream_SDO, Signals->SDO_Readvalue, i);
			//THE FOLLOWING CODE IS KEPT BECAUSE A BUG IN "GPIO_check()" FUNCTION.
			if(Stream_SDO[i] == 'R')
			{
				//Change GPIO to INPUT mode
				EMU_GPIO_SetMode(&(ID_SENSOR->SDO_signal), INPUT);

				//Check GPIO port input data register
				result = ID_SENSOR->SDO_signal.PORT->IDR & ID_SENSOR->SDO_signal.PIN;
				//Save the read value in SignalsValue structure
				if(result == 0)	Signals->SDO_Readvalue[SDO_read_counter] = 0;
				else Signals->SDO_Readvalue[SDO_read_counter] = 1;
				SDO_read_counter++;
			}

			//Delay to stabilize the signal
			/*__asm("nop");
			__asm("nop");
			__asm("nop");
			__asm("nop");
			__asm("nop");*/

			for(timming_counter = 0; timming_counter < ID_SENSOR->TIMMING_pulse; timming_counter++){
				__asm("nop");
			}
		}
		return true;
	}
	else
		return false;
}

/*****************************************************************************
 * Function name    : EMU_GPIO_SetupSensor
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
void EMU_GPIO_SetupSensor(SensorType *SENSOR, EMU_GPIO_Signal Signal1, EMU_GPIO_Signal Signal2, EMU_GPIO_Signal Signal3,
		EMU_GPIO_Signal Signal4, uint16_t WPULSE)
{
	SENSOR->CS_signal = Signal1;
	SENSOR->SCK_signal = Signal2;
	SENSOR->SDI_signal = Signal3;
	SENSOR->SDO_signal = Signal4;
	SENSOR->TIMMING_pulse = WPULSE;
}


/*****************************************************************************
 * Function name    : GPIO_check
 * 	  @brief		: Function that check and configure a GPIO signal status
 *
 *    @param1       : signal: EMU_GPIO_Signal that mapping a specific signal
 *    @param2       : genericStream: pointer to a Stream
 *    @param3		: ReadValue: pointer to array where save the value read
 *    @param4		: index: counter for the Stream array
 *
 * 	  @retval       : void
 *
 * 	  Notes         :
 *****************************************************************************/
void GPIO_check(EMU_GPIO_Signal signal, char *genericStream, char *ReadValue, uint8_t index)
{
	uint16_t signal_read_counter = 0, timming_counter = 0;
	uint32_t result = 0;


	if(genericStream[index] == '1')
	{
		//Check GPIO port output type and reconfigure GPIO mode (redundancy)
		if((signal.PORT->OTYPER & signal.PIN) == signal.PIN)
			EMU_GPIO_SetMode(&(signal), OPENDRAIN);
		else
			EMU_GPIO_SetMode(&(signal), OUTPUT);

		signal.PORT->BSRR = (1 << signal.numberPIN); //GPIO port bit set

	}

	else if(genericStream[index] == '0')
	{
		//Check GPIO port output type and reconfigure GPIO mode (redundancy)
		if((signal.PORT->OTYPER & signal.PIN) == signal.PIN)
			EMU_GPIO_SetMode(&(signal), OPENDRAIN);
		else
			EMU_GPIO_SetMode(&(signal), OUTPUT);

		signal.PORT->BSRR = (GPIO_BSRR_BR0 << (signal.numberPIN)); //GPIO port bit reset

	}

	//HERE, THERE IS A BUG. THE RESULT VALUE IS NOT CORRECT WHEN A GPIO STATUS IS READ
	/*else if(genericStream[index] == 'R')
	{
		//Change GPIO to INPUT mode
		EMU_GPIO_SetMode(&(signal), INPUT);

		//Check GPIO port input data register
		result = (signal.PORT->IDR) & (signal.PIN);
		//Save the read value in SignalsValue structure
		if(result == 0)	ReadValue[signal_read_counter] = 0;
		else ReadValue[signal_read_counter] = 1;
		signal_read_counter++;
	}

	for(timming_counter = 0; timming_counter < 50; timming_counter++){
		__asm("nop");
	}*/

}

