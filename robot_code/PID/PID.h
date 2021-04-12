#pragma once

class PID
{
public:
	PID(float Kp, float Ki, float Kd, float position_weight, float angle_weight);
	void clear();
	float update(float position_error, float angle_error);
	float setKp(float p);
	float setKi(float p);
	float setKd(float p);
private:
	float Kp, Ki, Kd;
	float ITerm;
	float p_weight, a_weight;
	float last_error;
};