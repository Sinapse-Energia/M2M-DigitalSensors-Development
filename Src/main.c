/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>  // provisional atoi()
#include <stdarg.h>
#include <time.h>  // provionale time()

#include "main.h"
#include "stm32f2xx_hal.h"
#include "M95lite.h"
#include "Definitions.h"
#include "southbound_ec.h"
#include "MQTTAPI.H"
#include "NBinterface.h"
#include "BLInterface.h"


#include "circular.h"
#include "utils.h"

#include "dma.h"		// BYDMA
#include "CAN_Util.h"



TIM_HandleTypeDef    TimHandle;
TIM_OC_InitTypeDef sConfig;
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


IWDG_HandleTypeDef hiwdg;


DMA_HandleTypeDef hdma_usart6_rx;

UART_HandleTypeDef huartDummy;
/// It is defined a variable but it is not going to be used.


/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/*********************************************************************
 * PRIVATE VARIABLES TO DEBUG Digital-Sensors_Development CODE
 */
#include "GPIO_wrapper.h"

/* The purpose of the following main code is testing a specific SHT10 Temperature & Humidity sensor
 * Datasheet is here: https://www.sparkfun.com/datasheets/Sensors/SHT1x_datasheet.pdf
 */

EMU_GPIO_Signal CS_1;
EMU_GPIO_Signal SCK_1;
EMU_GPIO_Signal SDI_1;
EMU_GPIO_Signal SDO_1;
uint16_t TIMMING_1 = 50;
SensorType SENSOR1;
SignalsValue SENSOR1_SIGNALS_RH;
SignalsValue SENSOR2_SIGNALS_RH;
SignalsValue SENSOR3_SIGNALS_RH;
SignalsValue SENSOR1_SIGNALS_TA;
SignalsValue SENSOR2_SIGNALS_TA;
SignalsValue SENSOR3_SIGNALS_TA;


/* According the datasheet sensor, we have to emulate Clock and Data signals in order to get a properly
 * communication with the sensor. In our case, we have divided the complete signal sequence in three separated
 * parts as follow:
 *
 * 	- Part1: Build the Start Transmission sequence and Send Command
 * 	- Part2: Receive the response from sensor (2 bytes)
 * 	- Part3: Receive the CRC checksum
 *
 * 	The sequence implementation is a programmer choice. It's not mandatory to use this testing example.
 */

/*CASE 1:  MEASURE TEMPERATURE*/
/*Build the sequence for Part1. In our case, we will only use SCK and SDO signals.
 * CS and SDI signals must be defined as "XXXX...." string.
 */
char *Stream_CS_RH_1 =  "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
char *Stream_SDI_RH_1 = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
char *Stream_SCK_RH_1 = "XXXXXXXX1101100010101010101010100010";		//Start Transmission and Clock signal
char *Stream_SDO_RH_1 = "11111111100011000XXXXXXXX1X0X1XXXXXX";		//Start Transmission and Send Command "Measure Relative Humidity"

//Build the sequence for Part 2. In our case, we have divided the sensor response in 2 separated bytes
//SCK and SDO signals are the same in both sequence
char *Stream_SDI_RH_1_partB = "XXXXXXXXXXXXXXXXXXXX";
char *Stream_CS_RH_1_partB =  "XXXXXXXXXXXXXXXXXXXX";
char *Stream_SCK_RH_1_partB = "1010101010101010XX10";
char *Stream_SDO_RH_1_partB = "RXRXRXRXRXRXRXRXX000";

//Build the Part 3: CRC checksum sequence
char *Stream_SDO_RH_1_partCHK = "RXRXRXRXRXRXRXRXXXXX";



/*CASE 2:  MEASURE RH.
 * Build the sequence for Part1. In our case, we will only use SCK and SDO signals.
 * CS and SDI signals must be defined as "XXXX...." string.
 */
char *Stream_CS_TA_1 =  "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
char *Stream_SDI_TA_1 = "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX";
char *Stream_SCK_TA_1 = "XXXXXXXX1101100010101010101010100010";		//Start Transmission and Clock signal
char *Stream_SDO_TA_1 = "11111111100011000XXXXXXXXXX1XXXXXXXX";		//Start Transmission and Send Command "Measure Relative Temperature"

//Build the sequence for Part 2. In our case, we have divided the sensor response in 2 separated bytes
//SCK and SDO signals are the same in both sequence
char *Stream_SDI_TA_1_partB = "XXXXXXXXXXXXXXXXXXXX";
char *Stream_CS_TA_1_partB =  "XXXXXXXXXXXXXXXXXXXX";
char *Stream_SCK_TA_1_partB = "1010101010101010XX10";
char *Stream_SDO_TA_1_partB = "RXRXRXRXRXRXRXRXX000";

//Build the Part 3: CRC checksum sequence
char *Stream_SDO_TA_1_partCHK = "RXRXRXRXRXRXRXRXXXXX";
/*****************************************************************************/


/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


int main(void)
{

  /* USER CODE BEGIN 1 */
	uint8_t value = 0;
	uint8_t  readwrite= 0;
	float Temperature = 0;				//Variable to store the Temperature value read
	float RelativeHumidity = 0;			//Variable to store the RH read
  /* USER CODE END 1 */
  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
 // MX_GPIO_Init();
  /* USER CODE BEGIN 2 */

  	/*In our test EVB, the sensor pins are connected as follow
  	 * This could change in other EVB or PCB
  	 */
	value = EMU_GPIO_config(&CS_1,PB1, OUTPUT,UP);
	value = EMU_GPIO_config(&SCK_1,PC8, OUTPUT,NONE);
	value = EMU_GPIO_config(&SDI_1,PB1, OUTPUT,UP);
	value = EMU_GPIO_config(&SDO_1,PB14, OPENDRAIN,UP);

  //HAL_Delay(1000);

 //Now, we register our sensor as SENSOR1
  config_Sensor(&SENSOR1, CS_1, SCK_1, SDI_1, SDO_1, TIMMING_1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

	  //We are going to get the RH value from sensor. We will use the EMU_GPIO_Read_Write function
	  // to emulate the proper signals
	  //First, emulate Part 1 signals
		readwrite = EMU_GPIO_Read_Write(&SENSOR1, Stream_CS_RH_1, Stream_SDI_RH_1, Stream_SDO_RH_1, Stream_SCK_RH_1, &SENSOR1_SIGNALS_RH);
		//As datasheet indicates, the RH measurement takes a maximum of 80ms
		HAL_Delay(90);

		//Then, emulate Part 2
		//The first byte response from sensor are saved in SENSOR_SIGNALS1_RH->SDO_Readvalue
		readwrite = EMU_GPIO_Read_Write(&SENSOR1, Stream_CS_RH_1_partB, Stream_SDI_RH_1_partB, Stream_SDO_RH_1_partB, Stream_SCK_RH_1_partB, &SENSOR1_SIGNALS_RH);
		//The second byte response from sensor are saved in SENSOR_SIGNALS2_RH->SDO_Readvalue
		readwrite = EMU_GPIO_Read_Write(&SENSOR1, Stream_CS_RH_1_partB, Stream_SDI_RH_1_partB, Stream_SDO_RH_1_partB, Stream_SCK_RH_1_partB, &SENSOR2_SIGNALS_RH);
		//To finish, emulate CRC cheksum signal. In our case, the signal is emulated but not validation was implemented.
		readwrite = EMU_GPIO_Read_Write(&SENSOR1, Stream_CS_RH_1_partB, Stream_SDI_RH_1_partB, Stream_SDO_RH_1_partCHK, Stream_SCK_RH_1_partB, &SENSOR3_SIGNALS_RH);

		//Wait 3 seconds
		HAL_Delay(3000);

		//We are going to get the Temperature value from sensor. We will use the EMU_GPIO_Read_Write function
	    // to emulate the proper signals
		//First, emulate Part 1 signals
		readwrite = EMU_GPIO_Read_Write(&SENSOR1, Stream_CS_TA_1, Stream_SDI_TA_1, Stream_SDO_TA_1, Stream_SCK_TA_1, &SENSOR1_SIGNALS_TA);
		//As datasheet indicates, the Temperature measurement takes a maximum of 320ms
		HAL_Delay(400);
		//Then, emulate Part 2
		//The first byte response from sensor are saved in SENSOR_SIGNALS1_TA->SDO_Readvalue
		readwrite = EMU_GPIO_Read_Write(&SENSOR1, Stream_CS_TA_1_partB, Stream_SDI_TA_1_partB, Stream_SDO_TA_1_partB, Stream_SCK_TA_1_partB, &SENSOR1_SIGNALS_TA);
		//The second byte response from sensor are saved in SENSOR_SIGNALS2_TA->SDO_Readvalue
		readwrite = EMU_GPIO_Read_Write(&SENSOR1, Stream_CS_TA_1_partB, Stream_SDI_TA_1_partB, Stream_SDO_TA_1_partB, Stream_SCK_TA_1_partB, &SENSOR2_SIGNALS_TA);
		//To finish, emulate CRC cheksum signal. In our case, the signal is emulated but not validation was implemented.
		readwrite = EMU_GPIO_Read_Write(&SENSOR1, Stream_CS_TA_1_partB, Stream_SDI_TA_1_partB, Stream_SDO_TA_1_partCHK, Stream_SCK_TA_1_partB, &SENSOR3_SIGNALS_TA);

		//Convert the 2 bytes read for Temperature value to a decimal representation
		Temperature = TemperatureMeasure (&(SENSOR1_SIGNALS_TA.SDO_Readvalue), &(SENSOR2_SIGNALS_TA.SDO_Readvalue));
		//Convert the 2 bytes read for RH value to a decimal representation
		RelativeHumidity = RHMeasure (&(SENSOR1_SIGNALS_RH.SDO_Readvalue), &(SENSOR2_SIGNALS_RH.SDO_Readvalue), Temperature);
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}



/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}



/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, blueRGB_Pin|redRGB_Pin|greenRGB_Pin|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|txDBG_3G_Pin|GPIO_PIN_9|GPIO_PIN_10
                          |emerg_3G_Pin|pwrKey_3G_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|Relay1_Pin|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : blueRGB_Pin redRGB_Pin greenRGB_Pin */
  GPIO_InitStruct.Pin = blueRGB_Pin|redRGB_Pin|greenRGB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : status_3G_Pin netlight_3G_Pin */
  GPIO_InitStruct.Pin = status_3G_Pin|netlight_3G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC4 PC5 PC8
                           PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA4 PA5 PA6
                           PA7 txDBG_3G_Pin PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|txDBG_3G_Pin|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 Relay1_Pin PB14 PB15
                           PB3 PB4 PB5 PB6
                           PB7*/
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|Relay1_Pin|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PWM_sim_Pin */
  GPIO_InitStruct.Pin = PWM_sim_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PWM_sim_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : emerg_3G_Pin pwrKey_3G_Pin */
  GPIO_InitStruct.Pin = emerg_3G_Pin|pwrKey_3G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : rxDBG_3G_Pin */
  GPIO_InitStruct.Pin = rxDBG_3G_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(rxDBG_3G_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
