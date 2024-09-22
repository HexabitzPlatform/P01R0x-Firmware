/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : P01R0_timers.c
 Description   : Peripheral timers setup source file.

 Required MCU resources :

 >> Timer 16 for micro-sec delay.
 >> Timer 17 for milli-sec delay.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/*----------------------------------------------------------------------------*/
/* Configure Timers                                                              */
/*----------------------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStruct = {0};
TIM_HandleTypeDef htim16; /* micro-second delay counter */
TIM_HandleTypeDef htim17; /* milli-second delay counter */

extern TIM_HandleTypeDef htim1;  /* Timer for RGB green */
extern TIM_HandleTypeDef htim2;  /* Timer for RGB red */
extern TIM_HandleTypeDef htim3;  /* Timer for RGB blue */

IWDG_HandleTypeDef hiwdg;

extern void MX_TIM1_Init(void);
extern void MX_TIM2_Init(void);
extern void MX_TIM3_Init(void);


void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/*-----------------------------------------------------------*/

/* IWDG init function */
void MX_IWDG_Init(void){

	/* Reload Value = [(Time * 32 KHz) / (4 * 2^(pr) * 1000)] - 1
	 * RL = [(500 mS * 32000) / (4 * 2^1 * 1000)]  - 1 = 2000 - 1 = 1999
	 * timeout time = 500 mS
	 * Pre-scaler = 8
	 * Reload Value = 1999
	 *  */

	hiwdg.Instance = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
	hiwdg.Init.Window = IWDG_WINDOW_DISABLE;
	hiwdg.Init.Reload =1999;

	HAL_IWDG_Init(&hiwdg);

}
/* TIM1 init function */
void MX_TIM1_Init(void)
{

	/* USER CODE BEGIN TIM1_Init 0 */
	__TIM1_CLK_ENABLE();
	/* USER CODE END TIM1_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = (uint32_t )(HAL_RCC_GetSysClockFreq() / PWM_TIMER_CLOCK) - 1;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 1023;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim1);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig);
	HAL_TIM_PWM_Init(&htim1);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, _RGB_GREEN_TIM_CH);

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);
	/* Peripheral clock enable */
	__HAL_RCC_TIM1_CLK_ENABLE();
	/* TIM1 interrupt Init */
	HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
}
/* TIM2 init function */
void MX_TIM2_Init(void)
{

	/* USER CODE BEGIN TIM2_Init 0 */
	__TIM2_CLK_ENABLE();
	/* USER CODE END TIM2_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = (uint32_t )(HAL_RCC_GetSysClockFreq() / PWM_TIMER_CLOCK) - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1023;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim2);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim2,&sClockSourceConfig);
	HAL_TIM_PWM_Init(&htim2);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC,_RGB_RED_TIM_CH);

	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

	HAL_TIM_MspPostInit(&htim2);

	/* TIM2 clock enable */
	__HAL_RCC_TIM2_CLK_ENABLE();

	/* TIM2 interrupt Init */
	HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

}

/*-----------------------------------------------------------*/
/* TIM3 init function */
void MX_TIM3_Init(void)
{

	/* USER CODE BEGIN TIM3_Init 0 */
	__TIM3_CLK_ENABLE();
	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = (uint32_t )(HAL_RCC_GetSysClockFreq() / PWM_TIMER_CLOCK) - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 1023;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim3);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);
	HAL_TIM_PWM_Init(&htim3);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, _RGB_BLUE_TIM_CH);

	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */
	HAL_TIM_MspPostInit(&htim3);

	/* TIM3 clock enable */
	__HAL_RCC_TIM3_CLK_ENABLE();

	/* TIM3 interrupt Init */
	HAL_NVIC_SetPriority(TIM3_TIM4_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM3_TIM4_IRQn);

}

/*-----------------------------------------------------------*/

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(timHandle->Instance==TIM1)
	{
		/* USER CODE BEGIN TIM1_MspPostInit 0 */

		/* USER CODE END TIM1_MspPostInit 0 */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**TIM1 GPIO Configuration
        PA11 [PA9] ------> TIM1_CH4
		 */
		GPIO_InitStruct.Pin = _RGB_GREEN_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
		HAL_GPIO_Init(_RGB_GREEN_PORT, &GPIO_InitStruct);

		/* USER CODE BEGIN TIM1_MspPostInit 1 */

		/* USER CODE END TIM1_MspPostInit 1 */
	}
	else if(timHandle->Instance==TIM2)
	{
		/* USER CODE BEGIN TIM2_MspPostInit 0 */

		/* USER CODE END TIM2_MspPostInit 0 */
		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**TIM2 GPIO Configuration
	PA15     ------> TIM2_CH1
		 */
		GPIO_InitStruct.Pin = _RGB_RED_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF2_TIM2;
		HAL_GPIO_Init(_RGB_RED_PORT, &GPIO_InitStruct);

		/* USER CODE BEGIN TIM2_MspPostInit 1 */

		/* USER CODE END TIM2_MspPostInit 1 */
	}

	else if(timHandle->Instance==TIM3)
	{
		/* USER CODE BEGIN TIM3_MspPostInit 0 */

		/* USER CODE END TIM3_MspPostInit 0 */

		__HAL_RCC_GPIOA_CLK_ENABLE();
		/**TIM3 GPIO Configuration
	PA6     ------> TIM3_CH1
		 */
		GPIO_InitStruct.Pin = _RGB_BLUE_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
		HAL_GPIO_Init(_RGB_BLUE_PORT, &GPIO_InitStruct);

		/* USER CODE BEGIN TIM3_MspPostInit 1 */

		/* USER CODE END TIM3_MspPostInit 1 */
	}
}

/*-----------------------------------------------------------*/
/*  Micro-seconds timebase init function - TIM16 (16-bit)
 */
void TIM_USEC_Init(void){

	__TIM16_CLK_ENABLE();

	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 47;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 0XFFFF;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim16);

	HAL_TIM_Base_Start(&htim16);

}
/*-------------------------------------------------------------*/
/*  Milli-seconds timebase init function - TIM17 (16-bit)
 */
void TIM_MSEC_Init(void){

	__TIM17_CLK_ENABLE();
	htim17.Instance = TIM17;
	htim17.Init.Prescaler = 47999;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = 0xFFFF;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter = 0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim17);

	HAL_TIM_Base_Start(&htim17);
}

/*-----------------------------------------------------------*/

/* --- Load and start micro-second delay counter --- 
 */
void StartMicroDelay(uint16_t Delay){
	uint32_t t0 =0;

	portENTER_CRITICAL();

	if(Delay){
		t0 =htim16.Instance->CNT;

		while(htim16.Instance->CNT - t0 <= Delay){};
	}

	portEXIT_CRITICAL();
}

/*-----------------------------------------------------------*/

/* --- Load and start milli-second delay counter --- 
 */
void StartMilliDelay(uint16_t Delay){
	uint32_t t0 =0;

	portENTER_CRITICAL();

	if(Delay){
		t0 =htim17.Instance->CNT;

		while(htim17.Instance->CNT - t0 <= Delay){};
	}

	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
