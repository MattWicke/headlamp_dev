/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "mpu_util.h"
//#include <stdio.h>
#include <math.h>
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
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim21;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM21_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, 1000);
    return 0;
}

HAL_StatusTypeDef init_mpu6050()
{
    HAL_StatusTypeDef err = 0;
    uint8_t who = 0;

    /*
     * check to see if the mpu6050 is connected
     * this should return a value of '104'
     * for who if its working. Doesn't need to be
     * powered on for this to work
     */
    err = HAL_I2C_Mem_Read(
		   &hi2c1
		  , MPU_I2C_ADDRESS
		  , MPU_WHO_AM_I
		  , 1 // mem address size
		  , &who
		  , 1 // buff size
		  , 1000 // timeout
		  );
    if(err)
    	return err;

    /*
     * set the power register to on
     */
    uint8_t pwr_reg_val = 0;
    err = HAL_I2C_Mem_Write(
    		  &hi2c1
			, MPU_I2C_ADDRESS
			, MPU_PWR_MGMT_1
			, 1 // mem address size
			, &pwr_reg_val
			, 1 //buff size
			, 1000 // time out
			);
    if(err)
    	return err;
    /*
     * set the sample rate
     * sample rate = gyro rate / (1 + sample reg)
     * if DLPF is off gyro = 8khz if on gyro = 1khz
     */
    uint8_t sample_rate = 0;
    err = HAL_I2C_Mem_Write(
    		  &hi2c1
			, MPU_I2C_ADDRESS
			, MPU_SMPLRT_DIV
			, 1 // mem address size
			, &sample_rate
			, 1 //buff size
			, 1000 // time out
			);
    if(err)
    	return err;

    /*
     * configure the gyro
     * no self test XG_ST = 0 YG_ST = 0 ZG_ST = 0
     * lowest full range scale GFS_SEL = 0 => +-250 deg/sec
     */
    uint8_t gyro_config = 0;
    err = HAL_I2C_Mem_Write(
    		  &hi2c1
			, MPU_I2C_ADDRESS
			, MPU_GYRO_CONFIG
			, 1 // mem address size
			, &gyro_config
			, 1 //buff size
			, 1000 // time out
			);

    if(err)
    	return err;

    /*
     * configure the accelerometer
     * no self test XA_ST = 0 YA_ST = 0 ZA_ST = 0
     * lowest full range scale AFS_SEL = 0 => +-2g
     */
    uint8_t accel_config = 0;
    err = HAL_I2C_Mem_Write(
    		  &hi2c1
			, MPU_I2C_ADDRESS
			, MPU_ACCEL_CONFIG
			, 1 // mem address size
			, &accel_config
			, 1 //buff size
			, 1000 // time out
			);

    if(err)
    	return err;

    /*
     * configure the digital low pass filter
     */
    uint8_t dlpf_val = 0;
    err = HAL_I2C_Mem_Write(
    		  &hi2c1
			, MPU_I2C_ADDRESS
			, MPU_CONFIG
			, 1 // mem address size
			, &dlpf_val
			, 1 //buff size
			, 1000 // time out
			);
    if(err)
    	return err;

    printf("Initialization Complete: id %d\r\n", who);
    return 0;
}

HAL_StatusTypeDef get_accel(float* accel_data)
{
	/*
	 * there are 6x 1-byte registers on the mcu6050 for
	 * the accelerometer: X-high X-low Y-high Y-low Z-high Z-low
	 * this will pack the high and low bytes for each into a single
	 * int16
	 */

	HAL_StatusTypeDef err = 0;
	uint8_t accel_buffer[6];
	int16_t accel_vals[3]; // this should be signed?

    err = HAL_I2C_Mem_Read(
		   &hi2c1
		  , MPU_I2C_ADDRESS
		  , MPU_ACCEL_XOUT_H // start at x-high and read 6 after
		  , 1 // mem address size
		  , accel_buffer
		  , 6 // buff size
		  , 1000 // timeout
		  );
    if(err)
    	return err;

    accel_vals[0] = (uint16_t)(accel_buffer[0] << 8  | accel_buffer[1]);
    accel_vals[1] = (uint16_t)(accel_buffer[2] << 8  | accel_buffer[3]);
    accel_vals[2] = (uint16_t)(accel_buffer[4] << 8  | accel_buffer[5]);

    for(int ii = 0; ii < 3; ++ii)
    {
		accel_data[ii] = accel_vals[ii] / 16384.0;
    }

	return err;
}

HAL_StatusTypeDef get_gyro(float* gyro_data)
{
	HAL_StatusTypeDef err = 0;
	uint8_t gyro_buffer[6];
	int16_t gyro_vals[3]; // this should be signed?

    err = HAL_I2C_Mem_Read(
		   &hi2c1
		  , MPU_I2C_ADDRESS
		  , MPU_GYRO_XOUT_H // start at x-high and read 6 after
		  , 1 // mem address size
		  , gyro_buffer
		  , 6 // buff size
		  , 1000 // timeout
		  );
    if(err)
    	return err;

    gyro_vals[0] = (uint16_t)(gyro_buffer[0] << 8  | gyro_buffer[1]);
    gyro_vals[1] = (uint16_t)(gyro_buffer[2] << 8  | gyro_buffer[3]);
    gyro_vals[2] = (uint16_t)(gyro_buffer[4] << 8  | gyro_buffer[5]);

    for(int ii = 0; ii < 3; ++ii)
    {
		gyro_data[ii] = gyro_vals[ii] / 131.0;
    }

	return err;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint32_t time_a, time_b;
	  float accel_angle_y, angle_y = 0;
	  float accel_data[3];
	  float gyro_data[3];
	  int err = 0;
	  uint32_t adc_val;

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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM21_Init();
  MX_ADC_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start(&htim21);

	err = init_mpu6050();
	if(err)
	{
		printf("ERROR in init: %d", err);
		while(1);
	}
	get_accel(accel_data);
	angle_y  = 180  /  M_PI* atan2(accel_data[0],accel_data[2]);
	HAL_ADC_Start(&hadc);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  time_a = htim21.Instance->CNT;
//	  get_accel(accel_data);
//	  get_gyro(gyro_data);
//	  printf("xa: %f ya: %f za: %f  ", accel_data[0], accel_data[1], accel_data[2]);
//	  printf("xg: %f yg: %f zg: %f  ", gyro_data[0], gyro_data[1], gyro_data[2]);
//	  HAL_Delay(100);



	  time_b = htim21.Instance->CNT;
	  time_a = time_b - time_a;

	  HAL_GPIO_TogglePin( GPIOB, GPIO_PIN_3);
	  //printf("time %lu \r\n", time_a);
//		accel_angle_y  = 180  /  M_PI* atan2(accel_data[0],accel_data[2]);
//		angle_y = 0.98 * (angle_y + (gyro_data[1] * time_a / 1000.0)) + (.02 * accel_angle_y);
//		printf("angle %f \r\n", angle_y);

	  adc_val = HAL_ADC_GetValue(&hadc);
	  printf("%lu\r\n", adc_val);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_8B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM21 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM21_Init(void)
{

  /* USER CODE BEGIN TIM21_Init 0 */

  /* USER CODE END TIM21_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM21_Init 1 */

  /* USER CODE END TIM21_Init 1 */
  htim21.Instance = TIM21;
  htim21.Init.Prescaler = 32000;
  htim21.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim21.Init.Period = 65535;
  htim21.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim21.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim21) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim21, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim21, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM21_Init 2 */

  /* USER CODE END TIM21_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
