/*
 * Created on Thu Jan 26 2017 
 *
 * Copyright (c) 2017 Thomas Northall-Little
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
#include "motor_class.h"

//This assumes that motor_pwm has already been initialised to the correct 
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
