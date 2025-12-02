/* Encoder and Motor control API */
#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include <stdint.h>

/* Start encoder peripheral (non-blocking) */
void Encoder_Start(void);

/* Read encoder counts since last read (signed) */
int32_t Encoder_Read(void);

/* Motor (PWM) helpers using TIM1 channels PA8/PA9 */
void Motor_Init(void);
/* Set compare values for TIM1 channels (0..ARR) */
void Motor_SetDuty(uint16_t ch1, uint16_t ch2);

#endif /* __ENCODER_H */

