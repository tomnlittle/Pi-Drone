
#include "defines.h"
#include "PCA9685.h"
#include "drone_class.h"
#include "BNO055_Interface.h"
#include "PID.h"
#include <thread>

static bool booted = false;
double roll_value = 0.0; //these are used for talking between threads until i get that figured
double pitch_value = 0.0;
double yaw_value = 0.0;

double throttle = 0.00; 
//for range finding/ ESC calibration
void setThrottleRange(void);

void droneThread(Drone); //PID pitch, PID roll, Drone d);

void BNO055Thread(PID, PID, PID, BNO055_Interface);

double checkThreshold(double, double);

int main(void) {

	Drone d;

	//InitialiseBNO055
	BNO055_Interface bINT;

	try {
        bINT.InitialiseBNO055();
    } catch (const char* msg){
        cout << msg << endl;
        sleep(1);
        bINT.Reset();
    }

	//bINT.ManualCalibration();

	PID pitch;
	PID roll;	
	PID yaw;

	d.setLanded(false);

	booted = true;

	std::thread bT(BNO055Thread, roll, pitch, yaw, bINT);
	usleep(5000); //wait for first readings to come through, this minimizes wonky errors at bootup
	std::thread dT(droneThread, d);

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
	bT.join();

	return 0;
} 

void droneThread(Drone d){
	while(booted){
		d.updateMotors(roll_value, pitch_value, 0.0, throttle);
		//printf("\nDroneTHREAD \n");
		//printf("D Roll : %lf\n", roll.getPID());
		//printf("D Pitch : %lf \n\n", pitch.getPID());
		usleep(MAIN_LOOP_WAIT+MAIN_LOOP_WAIT);
	}
}

void BNO055Thread(PID roll, PID pitch, PID yaw, BNO055_Interface b){
	double euler_array[EULER_DATA_ARRAY];
	for(int i = 0; i < EULER_DATA_ARRAY; i++)
		euler_array[i] = 0.0;

	while(booted){
		try {
			b.getEulerData(euler_array);
		} catch (const char* msg){
			cout << msg << endl;
			usleep(MAIN_LOOP_WAIT);
			continue;
		}
		
		roll.updatePID(3.141593, euler_array[2]);
		pitch.updatePID(-0.000000, euler_array[1]);
		yaw.updatePID(0.0, euler_array[0]);

		roll_value = checkThreshold(roll_value, roll.getPID());
		pitch_value = checkThreshold(pitch_value, pitch.getPID());
		yaw_value = 0.0;

		printf("Front Left at %lf \n", throttle - pitch_value + roll_value - (YAW_DIRECTION * yaw_value));
		printf("Front Right at %lf \n", throttle - pitch_value - roll_value + (YAW_DIRECTION * yaw_value));
		printf("Back Left at %lf \n", throttle + pitch_value + roll_value + (YAW_DIRECTION * yaw_value));
		printf("Back Right at %lf \n\n",throttle + pitch_value - roll_value - (YAW_DIRECTION * yaw_value));
		printf ("____________________________________ \n");

		//printf("PID Roll : %lf\n", roll.getPID());
		//printf("PID Pitch : %lf \n\n", pitch.getPID());		

		//usleep(MAIN_LOOP_WAIT); 
	}
}

double checkThreshold(double old_value, double new_value){
	//if((old_value + new_value)/2 > (old_value + 5.00)){
	if((old_value + 5) < new_value){
		return old_value;
	}

	return new_value;
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



