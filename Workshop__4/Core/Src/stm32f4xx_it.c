/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
typedef enum {
    POTENTIOMETER_ORANGE = 0,
    POTENTIOMETER_RED,
    POTENTIOMETER_BLUE,
    POTENTIOMETER_GREEN,
    POTENTIOMETER_MAX
} Potentiometer;
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_CHANNEL_1           ((uint32_t)ADC_CR1_AWDCH_0)
#define ADC_CHANNEL_2           ((uint32_t)ADC_CR1_AWDCH_1)
#define ADC_CHANNEL_3           ((uint32_t)(ADC_CR1_AWDCH_1 | ADC_CR1_AWDCH_0))
#define ADC_CHANNEL_4           ((uint32_t)ADC_CR1_AWDCH_2)
#define ADC_THRESHOLD			200
#define ADC_MAX				4095
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
static bool Light_status = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void updateDuty_cycle(uint8_t currentPotentiometer,  uint32_t adcValue);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
/* USER CODE BEGIN EV */
extern TIM_HandleTypeDef htim4;
/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line0 interrupt.
  */
void EXTI0_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI0_IRQn 0 */

  static uint32_t last_interrupt_time = 0;
  uint32_t current_time = HAL_GetTick();

  if(current_time - last_interrupt_time > 200)
  {
	Light_status = !Light_status;
	last_interrupt_time = current_time;
  }
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(BTN_LIGHT_Pin);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles ADC1, ADC2 and ADC3 global interrupts.
  */
void ADC_IRQHandler(void)
{
  static uint8_t currentPotentiometer = POTENTIOMETER_ORANGE; // Variable to track the current potentiometer

  uint32_t adcValue;
  // Read ADC value from the current potentiometer
  adcValue = HAL_ADC_GetValue(&hadc1);

  // Decide whether or not to light the LED on the selected channel
  if(Light_status)
  {
    if (adcValue <= ADC_THRESHOLD) 
    { 
      switch (currentPotentiometer)
      {
   	case POTENTIOMETER_ORANGE:
   		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
   		break;
        case POTENTIOMETER_RED:
        	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
        	break;
        case POTENTIOMETER_BLUE:
        	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
        	break;
        case POTENTIOMETER_GREEN:
        	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
        	break;
        default:
        	break;
      }
    } else if(adcValue < ADC_MAX - ADC_THRESHOLD)
    {
    	switch (currentPotentiometer) 
      {
    	  case POTENTIOMETER_ORANGE:
    	  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
 	  	updateDuty_cycle(currentPotentiometer, adcValue);
    	  	break;
    	  case POTENTIOMETER_RED:
    	  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	  	updateDuty_cycle(currentPotentiometer, adcValue);
    	  	break;
    	  case POTENTIOMETER_BLUE:
    	  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	  	updateDuty_cycle(currentPotentiometer, adcValue);
    	  	break;
    	  case POTENTIOMETER_GREEN:
    	  	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	  	updateDuty_cycle(currentPotentiometer, adcValue);
    	  	break;
    	  default:
    	  	break;
    	}
    } else // Completely ON state for a channel
    {
  	  updateDuty_cycle(currentPotentiometer, ADC_MAX);
    }
	  
    // Increment current potentiometer variable for the next iteration
    currentPotentiometer++;
    if (currentPotentiometer > POTENTIOMETER_MAX) 
    {
      currentPotentiometer = POTENTIOMETER_ORANGE;
    }

    ADC_ChannelConfTypeDef sConfig = {0};
    // Configure ADC channel for the next potentiometer
    sConfig.Channel = ADC_CHANNEL_1 + (currentPotentiometer - 1); // Adjusting currentPotentiometer to match ADC_CHANNEL
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
    // Configure ADC channel
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) 
    {
      Error_Handler();
    }
  } else // LEDs turned off from user button
  {
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_4);
  }

    HAL_ADC_Start_IT(&hadc1);
    HAL_ADC_IRQHandler(&hadc1);
}

/* USER CODE BEGIN 1 */
void updateDuty_cycle(uint8_t currentPotentiometer,  uint32_t adcValue)
{
    uint32_t dutyCycle;
    // Calculate duty cycle based on ADC value
    dutyCycle = (adcValue * htim4.Instance->ARR) / ADC_MAX;
    // Set duty cycle based on the current potentiometer
    switch (currentPotentiometer)
    {
      case POTENTIOMETER_ORANGE:
      TIM4->CCR1 = dutyCycle;
      break;
      case POTENTIOMETER_RED:
      TIM4->CCR2 = dutyCycle;
      break;
      case POTENTIOMETER_BLUE:
      TIM4->CCR3 = dutyCycle;
      break;
      case POTENTIOMETER_GREEN:
      TIM4->CCR4 = dutyCycle;
      break;
      default:
      break;
     }
}
/* USER CODE END 1 */
