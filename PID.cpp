#include "PID.h"

PID::PID(){
    current_time = clock();
    windupGuard = PID_WINDUPGUARD; //magic number
	P = PID_PROPORTIONAL;
	I = PID_INTEGRAL;
	D = PID_DERIVATIVE;
	previousPIDTime = current_time;
	integratedError = 0.0;
	lastError = 0.0;
    PIDValue = 0.0;
}

PID::~PID(){

}

//the magic balance function
//changes current motor speeds, pitch, yaw, roll in accordance with data
//The control algorithm so euler/quaternion values are matched by the motors
void PID::updatePID(double target, double current){
    double pTime = (((double)current_time) - ((double)previousPIDTime))/CLOCKS_PER_SEC;

    double error = target - current;

  /*  if(d.isLanded()){
    
        integratedError = 0.0;
    } else {
        integratedError += error * pTime;
    } */

    integratedError += error * pTime;

	//printf("time : %lf\n", pTime);
	//printf("error : %lf\n", integratedError);

    previousPIDTime = constrain(integratedError, -(windupGuard), windupGuard);
    
    double diffTerm = D * (current - lastError)/(pTime * 100);
    lastError = current;
    previousPIDTime = current_time;

    current_time = clock();
/*
    printf("diffTerm     : %lf\n", diffTerm);
    printf("Integral     : %lf\n", (I * integratedError));
    printf("Proportional : %lf\n", (P * error));
*/

    PIDValue = (P * error) + (I * integratedError) + diffTerm;
}

double PID::getPID(){
    return PIDValue;
}

void PID::PrintValues(){
   
}