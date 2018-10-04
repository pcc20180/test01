

#ifndef PID_H_
#define PID_H_

#define DELTA_T 0.004

class Pid
{
private:
	float KP,KI,KD;
	float diff[2];
	float integral;


public:
	explicit Pid(float kp,float ki, float kd);
	int Caluculation(int sensorvalue ,int targetvalue);
	~Pid(void);
};
Pid::Pid(float kp,float ki,float kd)
{
	Pid::KP = kp;
	Pid::KI = ki;
	Pid::KD = kd;
	Pid::integral = 0;
}

Pid::~Pid(void)
{
}

int Pid::Caluculation(int sensorvalue,int targetvalue)
{
	float p,i,d;
	Pid::diff[0] = Pid::diff[1];
	Pid::diff[1] = (sensorvalue - targetvalue);
	Pid::integral += (Pid::diff[1] + Pid::diff[0]) / 2.0 * DELTA_T;

	p = Pid::KP * Pid::diff[1];
	i = Pid::KI * Pid::integral;
	d = Pid::KD * (Pid::diff[1] - Pid::diff[0]) / DELTA_T;

	return p +i +d;

}


#endif //PID_H_
