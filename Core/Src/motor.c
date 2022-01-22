/*
 * stepper_motor.c
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

#include "motor.h"

void stepper_initialise(Stepper_Motor *dev, TIM_HandleTypeDef *timHandle,
		GPIO_TypeDef *dir_port, uint16_t dir_pin) {

	/* Set struct parameters */
	dev->timHandle = timHandle;
	dev->dir_port = dir_port;
	dev->dir_pin = dir_pin;
	HAL_GPIO_WritePin(dev->dir_port, dev->dir_pin, 0);
	if (timHandle->Instance == TIM15) {
		HAL_TIM_PWM_Start(dev->timHandle, TIM_CHANNEL_2);
	} else {
		HAL_TIM_PWM_Start(dev->timHandle, TIM_CHANNEL_1);
	}
}

void servo_initialise(Servo_Motor *dev, TIM_HandleTypeDef *timHandle) {
	/* Set struct parameters */
	dev->timHandle = timHandle;
	HAL_TIM_PWM_Start(dev->timHandle, TIM_CHANNEL_1);
}

void set_pwm(TIM_HandleTypeDef *TIM_pwm, double freq, float duty_cycle) {
	/*
	 set_pwm does set pwm timer to your specific value.

	 :param freq = frequency of pwm
	 :param duty_cycle is % duty cycle 0.0 - 1.0
	 :return: None
	 */
	uint16_t ARR_value = 1000000 / freq; //1000000 come from 275MHz/275
	uint16_t CCRx_value = ARR_value * duty_cycle;
	TIM_pwm->Instance->ARR = ARR_value;
	if (TIM_pwm->Instance == TIM15) {
		TIM_pwm->Instance->CCR2 = CCRx_value;
	} else {
		TIM_pwm->Instance->CCR1 = CCRx_value;
	}
}

void servo_set_degree(Servo_Motor *dev, uint8_t degree) {
	/*
	 servo_set_degree does set your servo to your given value.

	 :param degree is degree of servo motor (0-180)
	 :return: None
	 */
	if (degree > 180) {
		degree = 180;
	}
	else if (degree < 0){
		degree = 0;
	}
	float cyc = (degree / 180.0) * 0.2;
	set_pwm(dev->timHandle, 50, cyc);
}

void stepper_set_speed(Stepper_Motor *dev, double freq) {
	/*
	 stepper_set_speed does set your stepper to your given value.

	 :param freq can be -9999.9999 to 9999.9999. signed value use to set stepper direction.
	 :return: None
	 */
	if (freq > 0) {
		HAL_GPIO_WritePin(dev->dir_port, dev->dir_pin, 1);
		set_pwm(dev->timHandle, fabs(freq), 0.50);
	} else if (freq < 0) {
		HAL_GPIO_WritePin(dev->dir_port, dev->dir_pin, 0);
		set_pwm(dev->timHandle, fabs(freq), 0.50);
	} else {
		set_pwm(dev->timHandle, 100, 100.0);
	}
}
