
#include "defines.h"
#include "PCA9685.h"
#include "drone_class.h"
#include "BNO055_Interface.h"
#include "PID.h"
#include <thread>

static bool booted = false;
double roll_value; //these are used for talking between threads until i get that figured
double pitch_value;


//for range finding/ ESC calibration
void setThrottleRange(void);

void droneThread(double *throttle, Drone d); //PID pitch, PID roll, Drone d);

void BNO055Thread(PID roll, PID pitch, BNO055_Interface b);

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

	PID pitch;
	PID roll;	

	d.setLanded(false);

	double throttle = 10.00; 

	booted = true;

	std::thread bT(BNO055Thread, roll, pitch, bINT);
	std::thread dT(droneThread, &throttle, d);

	int hold_value;
	cin >> hold_value;
	booted = false;

	dT.join();
	bT.join();

	return 0;
} 

void droneThread(double *throttle, Drone d){
	while(booted){
		d.updateMotors(roll_value, pitch_value, 0.0, *throttle);
		//printf("\nDroneTHREAD \n");
		//printf("D Roll : %lf\n", roll.getPID());
		//printf("D Pitch : %lf \n\n", pitch.getPID());
		//usleep(MAIN_LOOP_WAIT+MAIN_LOOP_WAIT);
	}
}

void BNO055Thread(PID roll, PID pitch, BNO055_Interface b){
	double euler_array[EULER_DATA_ARRAY];
	for(int i = 0; i < EULER_DATA_ARRAY; i++)
		euler_array[i] = 0.0;

	while(booted){
		try {
			b.getEulerData(euler_array);
		} catch (const char* msg){
			cout << msg << endl;
			usleep(MAIN_LOOP_WAIT);
			//continue;
		}

		//printf("Yaw  : %lf \n", euler_array[0]);
		//printf("Pitch : %lf \n", euler_array[1]);
		//printf("Roll   : %lf \n", euler_array[2]);

		roll.updatePID(3.141593, euler_array[2]);
		pitch.updatePID(-0.000000, euler_array[1]);

		roll_value = roll.getPID();
		pitch_value = pitch.getPID();

		//printf("PID Roll : %lf\n", roll.getPID());
		//printf("PID Pitch : %lf \n\n", pitch.getPID());		

		usleep(MAIN_LOOP_WAIT); 
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



