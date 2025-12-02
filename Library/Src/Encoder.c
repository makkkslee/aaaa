#include "Encoder.h"

#include "tim.h"
#include "stm32f1xx_hal.h"

/* Start the configured encoder (TIM3) */
void Encoder_Start(void)
{
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
}

/* Read encoder counts since last read and clear counter. Return signed value. */
int32_t Encoder_Read(void)
{
	int32_t cnt = __HAL_TIM_GET_COUNTER(&htim3);
	if (cnt > 0x7fff) cnt -= 0x10000; /* convert to signed 16-bit */
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	return cnt;
}

/* Motor helpers using TIM1 channels PA8/PA9 */
void Motor_Init(void)
{
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void Motor_SetDuty(uint16_t ch1, uint16_t ch2)
{
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ch1);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ch2);
}

