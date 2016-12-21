
#include <exception>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <time.h>  

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
