/*
 * File: DC_MOTOR_cfg.c
 * Driver Name: [ DC MOTOR ]
 * SW Layer:   ECUAL
 * Created on: 15/2/2024
 * Author:     Abdallah Alnemr
 *
 */

#include "../Inc/DC_MOTOR.h"

const DC_MOTOR_CfgType DC_MOTOR_CfgParam[DC_MOTOR_UNITS] =
{
	// DC MOTOR 1 Configurations
    {
	    GPIOB,
		GPIOB,
		GPIO_PIN_0, //L298_IN1
		GPIO_PIN_1, //L298_IN2
		TIM2,
		TIM_CHANNEL_2, //L298_ENA
		72,
		DC_MOTOR_F_PWM,
		DC_MOTOR_PWM_RES
	}
};
