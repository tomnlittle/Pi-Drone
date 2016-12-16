
#include "motor_class.h"

//This assumes that motor_pwm has already been initialised to the corrent 
void Motor::init(int motor_pin, PCA9685 motor_pwm){
    pin = motor_pin;
    speed = 0.0;
    pwm = motor_pwm;
}

Motor::Motor(){}

Motor::~Motor(){}

int Motor::getSpeed(){
    return speed;
}

double Motor::getDutyCycle(){
    return raw_speed;
}

//converts the percentage of motor speed into a dutycycle value
void Motor::setSpeed(double new_speed){
	if(new_speed > 100.00 || new_speed < 0.00) return;
    double percent = new_speed/100.00;
	double dutyCycle = PWM_MIN + (percent*(PWM_MAX-PWM_MIN));
	pwm.setPWM(pin, 0, (int)dutyCycle);
    speed = new_speed;
    raw_speed = dutyCycle;
}
