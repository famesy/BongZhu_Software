/*
 * PID.h
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Output limits */
	float lim_min;
	float lim_max;

	/* Integrator limits */
	float lim_int_max;
	float lim_int_min;

	/* Controller "memory" */
	float proportional_term;
	float integrator;
	float integral_term;
	float derivative_term;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController;

#endif /* INC_PID_H_ */
