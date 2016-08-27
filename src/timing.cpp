#include "timing.h"

uint32_t delayMultiplier;


void initTiming() {
	__TIM3_CLK_ENABLE();

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	TIM_HandleTypeDef htim3;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	//htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim3);
	//HAL_TIM_Base_Start_IT(&htim3); // start timer interrupts

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

	delayMultiplier = HAL_RCC_GetSysClockFreq() / 12000000;
}

void delayMicros(uint32_t micros) {
	micros = micros * delayMultiplier - 10;

	while(micros--);
}



