/* PID controller class */
#ifndef _PID_H_
#define _PID_H_

#include "ros/ros.h"

class PID
{
public:
	PID(double Kp, double Ki, double Kd, double max_pid_output, double min_pid_output);
	~PID();
	double calculate_pid_output(double dt, double desired_reference, double sensor_measurement);
private:
	// Constructor variables
	double Kp;
	double Ki;
	double Kd;
	double max_pid_output;
	double min_pid_output;

	// PID errors and integrals
	double previous_error;
	double integral_error;
};

#endif