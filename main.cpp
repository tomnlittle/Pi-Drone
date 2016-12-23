
#include "defines.h"
#include "PCA9685.h"
#include "drone_class.h"
#include "BNO055_Interface.h"
#include "PID.h"

#include <chrono>


static bool booted = false;

double throttle = 0.00; 
//for range finding/ ESC calibration
void setThrottleRange(void);

void droneThread(); 

double checkThreshold(double, double);

int main(void) {

	printf("Starting... \n");

	booted = true;
	std::thread dT(droneThread);

	char hold_value = ' ';

	while(true){
		if(hold_value == 'w'){
			throttle += 2.00;
		} else if(hold_value == 's'){
			throttle -= 2.00;
		} else if (hold_value == 'd'){
			throttle = 0;
		}

		if(hold_value == 'h'){
			booted = false;
			break;
		}
		cin >> hold_value;
	} 

	dT.join();

	return 0;
} 

void droneThread(){
	Drone d;
	BNO055_Interface bINT;

	try {
        bINT.InitialiseBNO055();
    } catch (const char* msg){
        cout << msg << endl;
        sleep(1);
        bINT.Reset();
    }
	PID pitch;
	PID roll;	
	PID yaw;
	double roll_value = 0.0; //these are used for talking between threads until i get that figured
	double pitch_value = 0.0;
	double yaw_value = 0.0;
	d.setLanded(false);	
	while(booted){
		roll.updatePID(3.141593, bINT.getRoll());
		pitch.updatePID(-0.000000, bINT.getPitch());
		yaw.updatePID(0.0, bINT.getPitch());
		d.updateMotors(roll_value, pitch_value, yaw_value, throttle);
	}
}

//Only used for calibration procedures, completely works around class system
void setThrottleRange(void){
	PCA9685 pwm;
	pwm.init(I2C_ADAPTER, PCA9685_ADDRESS);
	pwm.setPWMFreq(PCA9685_FREQUENCY);
	sleep(2);
	printf("THROTTLE SET TO TOP\n");
	pwm.setPWM(MOTOR_0_PIN, 0, PWM_MAX);
	pwm.setPWM(MOTOR_1_PIN, 0, PWM_MAX);
	pwm.setPWM(MOTOR_2_PIN, 0, PWM_MAX);
	pwm.setPWM(MOTOR_3_PIN, 0, PWM_MAX);
	printf("PRESS ANY KEY TO SET MIN RANGE \n");
	int i;
	cin >> i;
	printf("MIN THROTTLE SET \n");
	pwm.setPWM(MOTOR_0_PIN, 0, PWM_MIN);
	pwm.setPWM(MOTOR_1_PIN, 0, PWM_MIN);
	pwm.setPWM(MOTOR_2_PIN, 0, PWM_MIN);
	pwm.setPWM(MOTOR_3_PIN, 0, PWM_MIN);
	printf("PRESS ANY KEY TO CONTINUE \n");
	cin >> i;
}



