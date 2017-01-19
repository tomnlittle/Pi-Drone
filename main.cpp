
#include "defines.h"
#include "PCA9685.h"
#include "drone_class.h"
#include "BNO055_Interface.h"
#include "PID.h"

#include <chrono>

#define PID_PROPORTIONAL                3.5
#define PID_INTEGRAL                    0.0
#define PID_DERIVATIVE                  0.0
#define PID_WINDUPGUARD                 0.375

static bool booted = false;
double throttle = 0.00; 
double PID_p;
double PID_i;
double PID_d;

//for range finding/ ESC calibration
void setThrottleRange(void);
void droneThread(); 
double checkThreshold(double, double);

int main(int argc, char **argv) {

	if(argc != 4){
		PID_d = 0;
		PID_i = 0;
		PID_p = 0;
	} else {
		sscanf (argv[1],"%lf",&PID_p);
		sscanf (argv[2],"%lf",&PID_i);
		sscanf (argv[3],"%lf",&PID_d);
	}

	printf("P : %lf 	I : %lf 	D : %lf \n", PID_p, PID_i, PID_d);

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

	printf("done...\n");

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
	pitch.init(PID_WINDUPGUARD, PID_p, PID_i, PID_d);
	PID roll;	
	roll.init(PID_WINDUPGUARD, PID_p, PID_i, PID_d);
	PID yaw;
	yaw.init(PID_WINDUPGUARD, 0, PID_i, PID_d);

	double hold_yaw = bINT.getYaw();
	
	d.setLanded(false);	
	while(booted){
		roll.updatePID(3.141593, bINT.getRoll());
		pitch.updatePID(2, bINT.getPitch());
		yaw.updatePID(hold_yaw, bINT.getYaw());

		//printf("yaw : %lf  -  roll : %lf  -  pitch : %lf\n", bINT.getYaw(), bINT.getRoll(), bINT.getPitch());
		//printf("PIDs 	Yaw : %lf  -  Roll : %lf  -  Pitch : %lf\n\n", yaw.getPID(), roll.getPID(), pitch.getPID());

		d.updateMotors(roll.getPID(), pitch.getPID(), yaw.getPID(), throttle);
		
		//sleep(1); csefvjsfkvkjsdnkvnsdkvnsdjkfnvkjsdnvkndsfkjvndskfvnksdnv
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



