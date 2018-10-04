#ifndef KYORI_H_
#define KYORI_H_

#include "Motor.h"
#define MASATU 1
class Kyori
{
private:
	ev3api::Motor& leftMotor;
	ev3api::Motor& rightMotor;
public:
	Kyori(ev3api::Motor& leftmotor,ev3api::Motor& rightmotor);
	~Kyori(void);
	int Kenti(int kyori);
	int Count(void);
	int Diff(void);
};

Kyori::Kyori(ev3api::Motor& leftmotor, ev3api::Motor& rightmotor)
	:leftMotor(leftmotor),
	rightMotor(rightmotor){
}

int Kyori::Kenti(int kyori){
	if((kyori * MASATU) < (leftMotor.getCount() + rightMotor.getCount())/2){
		return 1;
	}else{
		return 0;
	}
}

int Kyori::Count(void){
	return ((leftMotor.getCount() + rightMotor.getCount()) / 2 );
}

int Kyori::Diff(void){
	return (leftMotor.getCount() - (rightMotor.getCount()));
}

#endif //KYORI_H_
