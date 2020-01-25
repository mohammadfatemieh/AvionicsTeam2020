/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stdio.h"
#include "bmp280.h"
#include <strings.h>
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

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define BUG_LENGTH 512
char buffer[512]; // to store data
uint8_t spiData[2];

void delay_ms(uint32_t period_ms)
{
    HAL_Delay(period_ms);
}

//int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
//{
//
//    /* Implement the I2C write routine according to the target machine. */
//    return -1;
//}
//
///*!
// *  @brief Function for reading the sensor's registers through I2C bus.
// *
// *  @param[in] i2c_addr : Sensor I2C address.
// *  @param[in] reg_addr : Register address.
// *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
// *  @param[in] length   : No of bytes to read.
// *
// *  @return Status of execution
// *  @retval 0 -> Success
// *  @retval >0 -> Failure Info
// *
// */
//int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
//{
//
//    /* Implement the I2C read routine according to the target machine. */
//    return -1;
//}
//
///*!
// *  @brief Function for writing the sensor's registers through SPI bus.
// *
// *  @param[in] cs           : Chip select to enable the sensor.
// *  @param[in] reg_addr     : Register address.
// *  @param[in] reg_data : Pointer to the data buffer whose data has to be written.
// *  @param[in] length       : No of bytes to write.
// *
// *  @return Status of execution
// *  @retval 0 -> Success
// *  @retval >0 -> Failure Info
// *
// */

int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
//	uint8_t data[8], i;
//
//	for(i=0; i<length; i++) {
//		data[i*2]   = reg_addr + i;
//		data[i*2+1] = reg_data[i];
//	}

	// select the chip
	HAL_GPIO_WritePin(GPIOB, cs, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 10000);
	HAL_SPI_Transmit(&hspi2, reg_data, length, 10000);

	HAL_GPIO_WritePin(GPIOB, cs, GPIO_PIN_SET);
    return 0;
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] cs       : Chip select to enable the sensor.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data, uint16_t length)
{
	HAL_GPIO_WritePin(GPIOB, cs, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2, &reg_addr, 1, 10000);
	HAL_SPI_Receive(&hspi2, reg_data, length, 10000);

	HAL_GPIO_WritePin(GPIOB, cs, GPIO_PIN_SET);

    return 0;
}

/* to send the data to the uart */
void send_uart (char *string)
{
	uint8_t len = strlen (string);
	HAL_UART_Transmit(&huart1, (uint8_t *) string, len, 2000);  // transmit in blocking mode
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(const char api_name[], int8_t rslt)
{
	snprintf(buffer, sizeof(buffer), "%s\r\n", api_name);
    if (rslt != BMP280_OK)
    {
        if (rslt == BMP280_E_NULL_PTR)
        {
        	snprintf(buffer, sizeof(buffer), "Error [%d] : Null pointer error\r\n", rslt);
        }
        else if (rslt == BMP280_E_COMM_FAIL)
        {
        	snprintf(buffer, sizeof(buffer), "Error [%d] : Bus communication failed\r\n", rslt);
        }
        else if (rslt == BMP280_E_IMPLAUS_TEMP)
        {
        	snprintf(buffer, sizeof(buffer), "Error [%d] : Invalid Temperature\r\n", rslt);
        }
        else if (rslt == BMP280_E_DEV_NOT_FOUND)
        {
        	snprintf(buffer, sizeof(buffer), "Error [%d] : Device not found\r\n", rslt);
        }
        else
        {
            /* For more error codes refer "*_defs.h" */
        	snprintf(buffer, sizeof(buffer), "Error [%d] : Unknown error code\r\n", rslt);
        }
    }
    send_uart(buffer);
}

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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  int8_t rslt;
  struct bmp280_dev bmp;
  struct bmp280_config conf;
  struct bmp280_uncomp_data ucomp_data;
  uint32_t pres32, pres64;
  double pres;

  memset((void*) buffer, 0, sizeof(buffer));

  // Set CS to high, this is the "not selected" state
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);

  /* Map the delay function pointer with the function responsible for implementing the delay */
  bmp.delay_ms = delay_ms;

  /* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77))
  bmp.dev_id = BMP280_I2C_ADDR_PRIM;

   Select the interface mode as I2C
  bmp.intf = BMP280_I2C_INTF;

   Map the I2C read & write function pointer with the functions responsible for I2C bus transfer
  bmp.read = i2c_reg_read;
  bmp.write = i2c_reg_write;*/

  /* To enable SPI interface: comment the above 4 lines and uncomment the below 4 lines */

  bmp.dev_id = GPIO_PIN_4;
  bmp.read = spi_reg_read;
  bmp.write = spi_reg_write;
  bmp.intf = BMP280_SPI_INTF;

  rslt = bmp280_init(&bmp);
  print_rslt(" bmp280_init status", rslt);

  /* Always read the current settings before writing, especially when
   * all the configuration is not modified
   */
  rslt = bmp280_get_config(&conf, &bmp);
  print_rslt(" bmp280_get_config status", rslt);

  /* configuring the temperature oversampling, filter coefficient and output data rate */
  /* Overwrite the desired settings */
  conf.filter = BMP280_FILTER_COEFF_2;

  /* Pressure oversampling set at 4x */
  conf.os_pres = BMP280_OS_4X;

  /* Setting the output data rate as 1HZ(1000ms) */
  conf.odr = BMP280_ODR_1000_MS;
  rslt = bmp280_set_config(&conf, &bmp);
  print_rslt(" bmp280_set_config status", rslt);

  /* Always set the power mode after setting the configuration */
  rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
  print_rslt(" bmp280_set_power_mode status", rslt);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	/* Reading the raw data from sensor */
	rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

	/* Getting the compensated pressure using 32 bit precision */
	rslt = bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press, &bmp);

	/* Getting the compensated pressure using 64 bit precision */
	rslt = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press, &bmp);

	/* Getting the compensated pressure as floating point value */
	rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press, &bmp);
	snprintf(buffer, sizeof(buffer), "UP: %ld, P32: %ld, P64: %ld, P64N: %ld, P: %f\r\n",
	                 ucomp_data.uncomp_press,
	                 pres32,
	                 pres64,
	                 pres64 / 256,
	                 pres);
	send_uart(buffer);
	bmp.delay_ms(1000); /* Sleep time between measurements = BMP280_ODR_1000_MS */

	//PWM TEST
	for (int x = 0; x <= 100; x++) {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, x);

		char cycle;
		sprintf(cycle, "%d \r", TIM1->CCR1);
		HAL_UART_Transmit(&huart1, cycle, 64, 100);

		if (x == 0) {
			HAL_Delay(300);
		}

		HAL_Delay(500);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

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

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 800;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 100;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI_CS_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI_CS_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
void setupSensor(void)
{
 twiSend(address, 0x1E,1); //just send 1 byte that tells MS5611 to reset
 delay(20000); //delay 10 mS needed for device to execute reset
 for (int i=1;i<=6;i++)
 {
 twiReceive(address, 0xA0+i*2, 2); //read all 14 bytes for callibration data from PROM
 //printMsg("b0= 0x%x, b1= 0x%x, b2= 0x%x \n",buffer[0], buffer[1], buffer[2]); //for debug purposes
  delay(50); //at least 40 uS
 calibrationData[i] = buffer[0]<<8|buffer[1]; //pair of bytes goes into each element of callibrationData[i], global variables, 14 uint8_t into 7 uint16_t
  }
 //delay(50);
 //printMsg("b0= %d, b1= %d, b2= %d \n",buffer[0], buffer[1], buffer[2]); //for debug purposes
}

int getPressure(void)
{
 D1=0;D2=0;
 twiSend(address, 0x48,1); //set D1 OSR=4096 (overscan, maximum) 0x48
 delay(25000);//must be 15 mS or more
 twiReceive(address, 0x00, 3); //initiate and read ADC data, 3 bytes
 //printMsg("b0= 0x%x, b1= 0x%x, b2= 0x%x ===========\n",buffer[0], buffer[1], buffer[2]); //for debug purposes
 D1 = D1<<8 | buffer[0]; //shifting first MSB byte left
 D1 = D1<<8 | buffer[1]; //another byte
 D1 = D1<<8 | buffer[2]; //LSB byte last
 twiSend(address, 0x58,1); //set D2 OSR=4096 (overscan, maximum) 0x58
 delay(25000); //must be 15 mS or more
 twiReceive(address, 0x00, 3); //initiate and read ADC data, 3 bytes
 D2 = D2<<8 | buffer[0]; //shifting first MSB byte left
 D2 = D2<<8 | buffer[1]; //another byte
 D2 = D2<<8 | buffer[2]; //LSB byte last

 dT = D2 - ((int)calibrationData[5] << 8);
  TEMP = (2000 + (((int64_t)dT * (int64_t)calibrationData[6]) >> 23)); //temperature before second order compensation
  if (TEMP<2000)  //if temperature of the sensor goes below 20°C, it activates "second order temperature compensation"
    {
      T2=pow(dT,2)/2147483648;
      OFF2=5*pow((TEMP-2000),2)/2;
      SENS2=5*pow((TEMP-2000),2)/4;
      if (TEMP<-1500) //if temperature of the sensor goes even lower, below -15°C, then additional math is utilized
        {
          OFF2=OFF2+7*pow((TEMP+1500),2);
          SENS2=SENS2+11*pow((TEMP+1500),2)/2;
        }
    }
    else
      {
          T2=0;
          OFF2=0;
          SENS2=0;
      }
  TEMP = ((2000 + (((int64_t)dT * (int64_t)calibrationData[6]) >> 23))-T2); //second order compensation included
  OFF = (((unsigned int)calibrationData[2] << 16) + (((int64_t)calibrationData[4] * dT) >> 7)-OFF2); //second order compensation included
  SENS = (((unsigned int)calibrationData[1] << 15) + (((int64_t)calibrationData[3] * dT) >> 8)-SENS2); //second order compensation included
  P = (((D1 * SENS) >> 21) - OFF) >> 15;
  return P; //returns back pressure P
}
*/

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
