/*
 * PID.c
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

typedef struct {

	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Output limits */
	float limMin;
	float limMax;

	/* Integrator limits */
	float limMinInt;
	float limMaxInt;

	/* Controller "memory" */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */

	/* Controller output */
	float out;

} PIDController;
