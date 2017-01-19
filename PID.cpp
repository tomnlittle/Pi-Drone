#include "PID.h"

PID::PID(){
    current_time = clock();
	integratedError = 0.0;
	lastError = 0.0;
    PIDValue = 0.0;
    previousPIDTime = current_time;
}

void PID::init(double windup, double constant, double integral, double derivative){
    windupGuard = windup; 
	P = constant;
	I = integral;
	D = derivative;
}

PID::~PID(){

}

//the magic balance function
//changes current motor speeds, pitch, yaw, roll in accordance with data
//The control algorithm so euler/quaternion values are matched by the motors
void PID::updatePID(double target, double current){
    double pTime = (((double)current_time) - ((double)previousPIDTime))/CLOCKS_PER_SEC;

    double error = target - current;

    integratedError += 0.0;

    previousPIDTime = constrain(integratedError, -(windupGuard), windupGuard);
    
    double diffTerm = D * (current - lastError)/(pTime * 100);
    lastError = current;
    previousPIDTime = current_time;

    current_time = clock();

    PIDValue = (P * error) + (I * integratedError) + diffTerm;
}

double PID::getPID(){
    return PIDValue;
}

void PID::PrintValues(){
   
}