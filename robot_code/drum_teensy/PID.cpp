#include "PID.h"

void PID::clear()
{
	ITerm = 0;
}
PID::PID(float Kp, float Ki, float Kd, float position_weight, float speed_weight) :
	Kp(Kp), Ki(Ki), Kd(Kd), p_weight(position_weight), s_weight(speed_weight) 
{
}
float PID::update(float position_error, float speed_error)
{
	float current_error = p_weight * position_error + s_weight * speed_error;
	float delta_error = current_error - last_error;
	ITerm += current_error;
	float output = ITerm * Ki + current_error * Kp + delta_error * Kd;
	last_error = current_error;
	return output;
}
void PID::setKp(float p)
{
	Kp = p;
}
void PID::setKi(float i)
{
	Ki = i;
}
void PID::setKd(float d)
{
	Kd = d;
}