/*
 * File: HCSR04_cfg.c
 * Driver Name: [ HC-SR04 Ultrasonic Sensor ]
 * SW Layer:   ECUAL
 * Created on: 16/2/2024
 * Author:     Abdallah Alnemr
 *
 */

#include "../Inc/HCSR04.h"

const HCSR04_CfgType HCSR04_CfgParam[HCSR04_UNITS] =
{
	// HC-SR04 Sensor Unit 1 Configurations
    {
		GPIOA,         //Trigger pin1 -->> A11
		GPIO_PIN_11,
		TIM4,
		TIM_CHANNEL_1, //Echo pin1 -->> B6
		72
	}
};
