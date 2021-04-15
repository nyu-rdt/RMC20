#pragma once

class PID
{
public:
	PID(float Kp, float Ki, float Kd, float position_weight, float speed_weight);
	void clear();
	float update(float position_error, float speed_error);
	void setKp(float p);
	void setKi(float p);
	void setKd(float p);
private:
	float Kp, Ki, Kd;
	float ITerm;
	float p_weight, s_weight;
	float last_error;
};