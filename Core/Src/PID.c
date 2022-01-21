/*
 * PID.c
 *
 *  Created on: Jan 21, 2022
 *      Author: fame
 */

void PIDController_Init(PIDController *pid, float Kp, float Ki, float Kd,
		float lim_min, float lim_max, float lim_int_min, float lim_int_max) {
	/* Set Struct Variable */
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->lim_min = lim_min;
	pid->lim_min = lim_min;
	pid->lim_int_max = lim_int_max;
	pid->lim_int_min = lim_int_min;

	/* Clear controller variables */
	pid->proportional_term = 0.0;
	pid->integrator = 0.0;
	pid->integral_term = 0.0;
	pid->derivative_term = 0.0;
	pid->prevMeasurement = 0.0;
	pid->out = 0.0;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
	float error = setpoint - measurement;
	/*
	 * P term
	 */
	pid->proportional_term = pid->Kp * error;

	/*
	 * I term
	 */
	pid->integrator += error;
	pid->integral_term = pid->Ki * pid->integrator;
	/*
	 * Anti-wind-up
	 */
	if (pid->integral_term > pid->lim_int_max){
		pid->integral_term = pid->lim_int_max;
	}
	else if (pid->integral_term < pid->lim_int_min){
		pid->integral_term = pid->lim_int_min;
	}
	/*
	 * D term
	 */
	pid->derivative_term = pid->Kd * (measurement - pid->prevMeasurement);
	pid->prevMeasurement = measurement;
	pid->out = pid->proportional_term + pid->integral_term + pid->derivative_term;
	return pid->out;
}
