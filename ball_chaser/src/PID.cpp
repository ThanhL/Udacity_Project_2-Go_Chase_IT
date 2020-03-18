/* Simple PID controller*/
#include "pid_controller.h"

PID::PID(double Kp, double Ki, double Kd, double max_pid_output, double min_pid_output) :
	Kp(Kp),
	Ki(Ki),
	Kd(Kd),
	max_pid_output(max_pid_output),
	min_pid_output(min_pid_output),
	previous_error(0.0),
	integral_error(0.0)
{
}

PID::~PID()
{
}



double PID::calculate_pid_output(double dt, double desired_reference, double sensor_measurement)
{
	// Calculate the error at time t
	double error = desired_reference - sensor_measurement;

	// Update integral error
	integral_error += error * dt;

	// Calculate the derivative of error
	double derivative_error = (error - previous_error) / dt;

	// PID output
	double pid_output = (Kp * error) +			// Proportional term
					(Ki * integral_error) +		// Integral term
					(Kd * derivative_error);		// Derivative term

	// Constrain PID output
	if (pid_output > max_pid_output)
	{
		pid_output = max_pid_output;
	}
	else if (pid_output < min_pid_output)
	{
		pid_output = min_pid_output;
	} 

	// Update previous error
	previous_error = error;
}

