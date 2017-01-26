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