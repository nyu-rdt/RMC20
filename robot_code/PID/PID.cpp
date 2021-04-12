#include "PID.h"
void PID::clear()
{
	ITerm = 0;
}
PID::PID(float Kp, float Ki, float Kd, float position_weight, float angle_weight) :
	Kp(Kp), Ki(Ki), Kd(kd), p_weight(position_weight), a_weight(angle_weight) {}
float PID::update(float position_error, float angle_error)
{
	float current_error = p_weight * position_error + a_weight * angle_error;
	float delta_error = current_error - last_error;
	ITerm += current_error;
	float output = ITerm * Ki + current_error * Kp + delta_error * Kd;
	last_error = current_error;
	return output;
}
float PID::setKp(float p)
{
	Kp = p;
}
float PID::setKi(float p)
{
	Ki = i;
}
float PID::setKd(float p)
{
	Kd = d;
}