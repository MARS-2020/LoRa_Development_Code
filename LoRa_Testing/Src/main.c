/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lora.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void writeReg(uint8_t addr, uint8_t value);
uint8_t readReg(uint8_t addr);
void sendPacket(uint8_t data[]);
void writeReg_Burst(uint8_t addr, uint8_t data[], uint8_t length);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t send[] = "Hello Jacob\n";
uint8_t headerTo = 255;
uint8_t headerFrom = 255;
uint8_t headerID = 0;
uint8_t headerFlags = 0;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  //reset LoRa
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);
  HAL_Delay(10);

  //set to sleep mode
  writeReg(RH_RF95_REG_01_OP_MODE, (readReg(RH_RF95_REG_01_OP_MODE) & ~(0x3)));

  //set to HF mode, and long range mode
  uint8_t temp = readReg(RH_RF95_REG_01_OP_MODE) & ~(0x1 << 3); //HF mode
  temp = temp | (0x1 << 7); //long range mode
  writeReg(RH_RF95_REG_01_OP_MODE, temp);

  //set frequency to 915MHz = 0xE4C000
  writeReg(RH_RF95_REG_08_FRF_LSB, 0x00);
  writeReg(RH_RF95_REG_07_FRF_MID, 0xC0);
  writeReg(RH_RF95_REG_06_FRF_MSB, 0xE4);

  //set up FIFO
  writeReg(RH_RF95_REG_0E_FIFO_TX_BASE_ADDR, 0);
  writeReg(RH_RF95_REG_0F_FIFO_RX_BASE_ADDR, 0);

  //set to STDBY mode from IDLE
  writeReg(RH_RF95_REG_01_OP_MODE, (readReg(RH_RF95_REG_01_OP_MODE) | RH_RF95_MODE_STDBY));

  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	sendPacket(send);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_RST_Pin */
  GPIO_InitStruct.Pin = LORA_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RST_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//Function for writing to a register
void writeReg(uint8_t addr, uint8_t value)
{
	uint8_t reg = addr | 0x80;
	uint8_t val = value;
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); //pull NSS low to start frame
	HAL_SPI_Transmit(&hspi2, &reg, (uint16_t)sizeof(reg), 1000);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_SPI_Transmit(&hspi2, &val, (uint16_t)sizeof(val), 1000);
	while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET); //pull NSS high to end frame
}

//Function for reading from a register
uint8_t readReg(uint8_t addr)
{
	uint8_t reg = addr & ~0x80;
	uint8_t data = 0;
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); //pull NSS low to start frame
	//HAL_SPI_TransmitReceive(&hspi2, &reg, &data, 1, 1000);
	HAL_SPI_Transmit(&hspi2, &reg, sizeof(reg), 1000); //send a read command from that address
	HAL_SPI_Receive(&hspi2, &data, sizeof(data), 1000);
	HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET); //pull NSS high to end frame
	return data;
}

//Function to burst write (primarily for FIFO)
void writeReg_Burst(uint8_t addr, uint8_t data[], uint8_t length)
{
	uint8_t reg = addr | 0x80;
	uint8_t val = 0;
	if (length >= 1)
	{
		HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET); //pull NSS low to start frame
		HAL_SPI_Transmit(&hspi2, &reg, (uint16_t)sizeof(reg), 1000);
		while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
		for(int i = 0; i < length; i++)
		{
			val = data[i];
			HAL_SPI_Transmit(&hspi2, &val, (uint16_t)sizeof(val), 1000);
			while(HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY);
		}

		HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_SET); //pull NSS high to end frame
	}
}

//Function for sending a packet to another lora module (not finished)
void sendPacket(uint8_t data[])
{
	  //send information
	  	  //set mode idle
	  	  //position at beginning of FIFO
	  	  //headers
	  	  //message data
	  	  //payload length
	  	  //set mode to TX

	//set mode to idle
	uint8_t temp = readReg(RH_RF95_REG_01_OP_MODE) & ~(0x3);
	temp = temp | RH_RF95_MODE_STDBY;
	writeReg(RH_RF95_REG_01_OP_MODE, temp);

	//position pointer at the beginning of the FIFO
	writeReg(RH_RF95_REG_0D_FIFO_ADDR_PTR, 0);

	//set headers
	writeReg(RH_RF95_REG_00_FIFO, headerTo); //header TO
	writeReg(RH_RF95_REG_00_FIFO, headerFrom); //header FROM
	writeReg(RH_RF95_REG_00_FIFO, headerID); //header ID
	writeReg(RH_RF95_REG_00_FIFO, headerFlags); //header FLAGS

	uint8_t size = (sizeof(send)/sizeof(send[0]));

	//write message data to fifo
	writeReg_Burst(RH_RF95_REG_00_FIFO, send, size);

	//set payload length
	writeReg(RH_RF95_REG_22_PAYLOAD_LENGTH, sizeof(send) + RH_RF95_HEADER_LEN);

	HAL_Delay(10); //delay some time

	//set mode to TX
	temp = readReg(RH_RF95_REG_01_OP_MODE) & ~(0x3);
	temp = temp | RH_RF95_MODE_TX;

	HAL_Delay(1000); //delay some time
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
