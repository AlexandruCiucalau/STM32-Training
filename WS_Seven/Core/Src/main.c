/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dac.h"
#include <math.h>
#include "MY_LIS3DSH.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ANGLE_THRESHOLD					10
#define NUM_VALUES_TO_AVERAGE				8
#define DEGREES_90					90
#define DEGREES_180					180
#define DEGREES_270					270
#define DEGREES_360					360
#define X_TOLERANCE 					45.0
#define Y_TOLERANCE 					34.0


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

I2S_HandleTypeDef hi2s3;
DMA_HandleTypeDef hdma_spi3_tx;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
int16_t dataI2S[100] = {0};
enum LEDDirection {
    NORTH,
    EAST,
    SOUTH,
    WEST,
    FLAT
};

LIS3DSH_DataScaled myData;
uint8_t tiltedLed = 4;
static uint8_t drdyFlag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2S3_Init(void);
/* USER CODE BEGIN PFP */
uint8_t determineLED(LIS3DSH_DataScaled data);
void updateDutycycle(uint8_t led, LIS3DSH_DataScaled data);
bool isFlat(LIS3DSH_DataScaled data);
void setSound(uint8_t led, LIS3DSH_DataScaled data);
void acquireAndAverageData(LIS3DSH_DataScaled *newData);
void startPWM(uint8_t led);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_I2S3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim4);
  LIS3DSH_InitTypeDef myAccConfigDef;

  myAccConfigDef.dataRate = LIS3DSH_DATARATE_25;
  myAccConfigDef.fullScale = LIS3DSH_FULLSCALE_4;
  myAccConfigDef.antiAliasingBW = LIS3DSH_FILTER_BW_50;
  myAccConfigDef.enableAxes = LIS3DSH_XYZ_ENABLE;
  myAccConfigDef.interruptEnable = true;
  LIS3DSH_Init(&hspi1, &myAccConfigDef);

  LIS3DSH_X_calibrate(-1000.0, 980.0);
  LIS3DSH_Y_calibrate(-1020.0, 1040.0);
  LIS3DSH_Z_calibrate(-920.0, 1040.0);

  CS43L22_Init();

    //Transmit empty data
    HAL_I2S_Transmit_DMA(&hi2s3, (uint16_t *)dataI2S, 100);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
        myData = LIS3DSH_GetDataScaled();

        // Prepare dataRdy flag and average of coordinates
        acquireAndAverageData(&myData);

        if (drdyFlag)
        {
            // Determine LED direction
            tiltedLed = determineLED(myData);

            // Control LED based on direction
            startPWM(tiltedLed);
            setSound(tiltedLed, myData);
            updateDutycycle(tiltedLed, myData);
            drdyFlag = 0;
        }

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

  /** Macro to configure the PLL multiplication factor
  */
  __HAL_RCC_PLL_PLLM_CONFIG(16);

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2S3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S3_Init(void)
{

  /* USER CODE BEGIN I2S3_Init 0 */

  /* USER CODE END I2S3_Init 0 */

  /* USER CODE BEGIN I2S3_Init 1 */

  /* USER CODE END I2S3_Init 1 */
  hi2s3.Instance = SPI3;
  hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
  hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
  hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_48K;
  hi2s3.Init.CPOL = I2S_CPOL_LOW;
  hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S3_Init 2 */

  /* USER CODE END I2S3_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim4.Init.Period = 39;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PD4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PE0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool isFlat(LIS3DSH_DataScaled data)
{
    if (data.x > -X_TOLERANCE && data.x < X_TOLERANCE &&
        data.y > -Y_TOLERANCE && data.y < Y_TOLERANCE) {
            // The board is considered flat
            return true;
        }

    // The board is not flat
    return false;
}


float calculateTiltAngle(LIS3DSH_DataScaled data) {
    return atan2(data.y, data.x) * 180.0 / M_PI;
}

void updateDutycycle(uint8_t led, LIS3DSH_DataScaled data)
{
    const float midpointAngles[4] = {DEGREES_90, DEGREES_180, DEGREES_270, DEGREES_360};
    uint32_t dutyCycle;
    float tiltAngle = calculateTiltAngle(data);

    // Normalize angle to be between 0 and 360 degrees
    tiltAngle = fmod(tiltAngle + DEGREES_360, DEGREES_360);
    // Calculate angle difference from midpoint
    float angleDifference = fabs(midpointAngles[led] - tiltAngle); // Adjust index to match array
    // Calculate duty cycle as a fraction of 90 degrees,for value between 0 and ARR
    dutyCycle = (uint32_t)((angleDifference / 90.0) * htim4.Instance->ARR);

    // Set duty cycle based on the current LED
    switch (led) {
        case NORTH:
            TIM4->CCR1 = dutyCycle;
            break;
        case EAST:
            TIM4->CCR2 = dutyCycle;
            break;
        case SOUTH:
            TIM4->CCR3 = dutyCycle;
            break;
        case WEST:
            TIM4->CCR4 = dutyCycle;
            break;
        case FLAT:
        	// Do nothing
            break;
    }

}

// Function to determine LED direction based on tilt angle
uint8_t determineLED(LIS3DSH_DataScaled data) {
    float tiltAngle = calculateTiltAngle(data);

    // Normalize angle to be between 0 and 360 degrees
    tiltAngle = fmod(tiltAngle + 360, 360);

    // Divide the range into four equal parts and assign LEDs

    if(isFlat(data))
    {
    	return FLAT;
    }

    if (tiltAngle >= 0 +  ANGLE_THRESHOLD && tiltAngle <= DEGREES_90) {
        return NORTH;
    } else if (tiltAngle >= DEGREES_90 +  ANGLE_THRESHOLD && tiltAngle <= DEGREES_180 ) {
        return EAST;
    } else if (tiltAngle >= DEGREES_180 +  ANGLE_THRESHOLD && tiltAngle <= DEGREES_270) {
        return SOUTH;
    } else if(tiltAngle >= DEGREES_270 +  ANGLE_THRESHOLD && tiltAngle <= DEGREES_360){
        return WEST;
    } else
    {
    	return FLAT;
    }
}

void setSound(uint8_t led,LIS3DSH_DataScaled data)
{
    float midpointAngles[4] = {DEGREES_90, DEGREES_180, DEGREES_270, DEGREES_360};
    float tiltAngle = calculateTiltAngle(data);

    // Normalize angle to be between 0 and 360 degrees
    tiltAngle = fmod(tiltAngle + DEGREES_360, DEGREES_360);
    // Calculate angle difference from midpoint
    float angleDifference = fabs(midpointAngles[led] - tiltAngle);

    if (led != FLAT)
    {
        if(angleDifference <= 45)
    	{
    		CS43L22_Beep(C5, 200);
    	}else
    	{
    		CS43L22_Beep(F5, 200);
    	}
    }else
    {
    	// Do nothing
    }
}

void acquireAndAverageData(LIS3DSH_DataScaled *newData) {
    static LIS3DSH_DataScaled accumulatedData = {0};
    static uint8_t numValuesAveraged = 0;

    // Accumulate new data
    accumulatedData.x += newData->x;
    accumulatedData.y += newData->y;
    accumulatedData.z += newData->z;

    numValuesAveraged++; // Increment the number of values averaged

    // Check if enough values are averaged
    if (numValuesAveraged >= NUM_VALUES_TO_AVERAGE) {
        // Calculate the average
        accumulatedData.x /= NUM_VALUES_TO_AVERAGE;
        accumulatedData.y /= NUM_VALUES_TO_AVERAGE;
        accumulatedData.z /= NUM_VALUES_TO_AVERAGE;

        // Data is ready
        drdyFlag = 1;

        // Update newData with averaged data
        newData->x = accumulatedData.x;
        newData->y = accumulatedData.y;
        newData->z = accumulatedData.z;

        // Reset counter and accumulated data for the next batch
        numValuesAveraged = 0;
        accumulatedData = (LIS3DSH_DataScaled){0};
    }
}

void startPWM(uint8_t led) {
    switch (led) {
        case NORTH:
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
            break;
        case EAST:
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
            break;
        case SOUTH:
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
            break;
        case WEST:
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
            HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
            break;
        case FLAT:
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
            HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
            break;
        default:
            // Do nothing
            break;
    }
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

