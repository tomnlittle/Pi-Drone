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
 
#include <exception>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>  
#include <thread>

#define DEBUG                           false

#define PWM_MAX 						670
#define PWM_MIN 						400
#define I2C_ADAPTER 			        1
#define PCA9685_ADDRESS					0x40
#define PCA9685_FREQUENCY   			61
#define YAW_DIRECTION                   -1

#define MOTOR_0_PIN  					0		
#define MOTOR_1_PIN						4
#define MOTOR_2_PIN						8
#define MOTOR_3_PIN						12

#define BNO055_ADDRESS                  0x28

#define SYSTEM_ERROR                    -1

#define EULER_DATA_ARRAY                3

#define MAIN_LOOP_WAIT                  500

using namespace std;
