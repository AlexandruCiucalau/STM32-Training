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
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern TIM_HandleTypeDef htim4;
extern unsigned int freqVal ;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
/* USER CODE BEGIN EV */

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
	//variable to count desired Frequency (in kHz)
	static uint8_t Freq = 5;
	uint16_t aux = 0;
	Freq += 5;

	if(Freq > 100)
	{
		Freq = 5;
	}
/*
    static uint32_t last_interrupt_time = 0;
    uint32_t current_time = HAL_GetTick();

    // Check if it's been at least 200 milliseconds since the last interrupt
    if ((current_time - last_interrupt_time) > 200) {
        Freq+=5;

        if (Freq >= 105) {
            Freq = 5;
        }
    }

    // Update last interrupt time
    last_interrupt_time = current_time;
*/
	//Calculating the new Counter Period for desired Frequency
    aux = 8000/Freq;
    aux = aux / 2;
    aux = aux - 1;
    htim4.Instance->ARR = aux;
    // Hardcoded implementation (Freq would have other values 0 - 19)
    /*if(Freq == 0)
    {
    	htim4.Instance->ARR = 799;
    }else if(Freq == 1)
    {
    	htim4.Instance->ARR = 399;
    }else if(Freq == 2)
    {
    	htim4.Instance->ARR = 266;
    }else if(Freq == 3)
    {
    	htim4.Instance->ARR = 199;
    }else if(Freq == 4)
    {
    	htim4.Instance->ARR = 159;
    }else if(Freq == 5)
    {
    	htim4.Instance->ARR = 132;
    }else if(Freq == 6)
    {
    	htim4.Instance->ARR = 113;
    }else if(Freq == 7)
    {
    	htim4.Instance->ARR = 99;
    }else if(Freq == 8)
    {
    	htim4.Instance->ARR = 87;
    }else if(Freq == 9)
    {
    	htim4.Instance->ARR = 79;
    }else if(Freq == 10)
    {
    	htim4.Instance->ARR = 71;
    }else if(Freq == 11)
    {
    	htim4.Instance->ARR = 65;
    }else if(Freq == 12)
    {
    	htim4.Instance->ARR = 60;
    }else if(Freq == 13)
    {
    	htim4.Instance->ARR = 56;
    }else if(Freq == 14)
    {
    	htim4.Instance->ARR = 52;
    }else if(Freq == 15)
    {
    	htim4.Instance->ARR = 49;
    }else if(Freq == 16)
    {
    	htim4.Instance->ARR = 46;
    }else if(Freq == 17)
    {
    	htim4.Instance->ARR = 43;
    }else if(Freq == 18)
    {
    	htim4.Instance->ARR = 41;
    }else if(Freq == 19)
    {
    	htim4.Instance->ARR = 39;
    }*/
    //Setting up duty cycles according to the now Counter Period
    TIM4->CCR1=(20.0/100.0)*htim4.Instance->ARR ;
    TIM4->CCR2=(40.0/100.0)*htim4.Instance->ARR ;
    TIM4->CCR3=(60.0/100.0)*htim4.Instance->ARR ;
    TIM4->CCR4=(60.0/100.0)*htim4.Instance->ARR ;
  /* USER CODE END EXTI0_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
  /* USER CODE BEGIN EXTI0_IRQn 1 */

  /* USER CODE END EXTI0_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
