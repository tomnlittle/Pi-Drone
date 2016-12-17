
#include "defines.h"
#include "PCA9685.h"
#include "drone_class.h"
#include "BNO055_Interface.h"

typedef struct PID {
	double P,I,D;
	clock_t previousPIDTime; // the last time the PID was updated
	double integratedError;
	double lastError;
	double windupGuard;
}PID;

clock_t current_time;

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

double updatePID(PID *p, double, double, Drone);

//for range finding/ ESC calibration
void setThrottleRange(void);

int main(void) {

	double euler_array[EULER_DATA_ARRAY];
	for(int i = 0; i < EULER_DATA_ARRAY; i++)
		euler_array[i] = 0.0;

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

	//PID yaw; 
	PID *pitch = new struct PID;
	PID *roll = new struct PID;

	current_time = clock();

	pitch->windupGuard = 0.375; //magic number
	pitch->P = 0.5;
	pitch->I = 0.5;
	pitch->D = 0.5;
	pitch->previousPIDTime = current_time;
	pitch->integratedError = 0.0;
	pitch->lastError = 0.0;

	roll->windupGuard = 0.375;
	roll->P = 0.5;
	roll->I = 0.5;
	roll->D = 0.5;
	roll->previousPIDTime = current_time;
	roll->integratedError = 0.0;
	roll->lastError = 0.0;

	double rollValue = 0.0;
	double pitchValue = 0.0;
	double yawValue = 0.0;

	d.setLanded(false);

	double throttle = 10; 

	while(true){
		d.updateMotors(rollValue, pitchValue, yawValue, throttle);
		try {
			bINT.getEulerData(euler_array);
		} catch (const char* msg){
			cout << msg << endl;
			//continue;
			break;
		}

		printf("Yaw  : %lf \n", euler_array[0]);
		printf("Pitch : %lf \n", euler_array[1]);
		printf("Roll   : %lf \n", euler_array[2]);

		current_time = clock();
		rollValue = updatePID(roll, 3.141593, euler_array[2], d);
		pitchValue = updatePID(pitch, -0.000000, euler_array[1], d);

		printf("PID Roll : %lf\n", rollValue);
		printf("PID Pitch : %lf \n\n", pitchValue);		

		usleep(500);
		
	}

	return 0;
} 

//the magic balance function
//changes current motor speeds, pitch, yaw, roll in accordance with data
//The control algorithm so euler/quaternion values are matched by the motors
double updatePID(PID *p, double target, double current, Drone d){
   

    double pTime = (((double)current_time) - ((double)p->previousPIDTime))/CLOCKS_PER_SEC;

    double error = target - current;

    if(d.isLanded()){
        p->integratedError = 0.0;
    } else {
        p->integratedError += error * pTime;
    }
	printf("time : %lf\n", pTime);
	printf("error : %lf\n", p->integratedError);

    p->previousPIDTime = constrain(p->integratedError, -(p->windupGuard), p->windupGuard);
    
    double diffTerm = p->D * (current - p->lastError)/(pTime * 100);
    p->lastError = current;
    p->previousPIDTime = current_time;

    return (p->P * error) + (p->I * p->integratedError) + diffTerm;
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



