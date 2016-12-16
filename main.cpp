
#include "defines.h"
#include "PCA9685.h"
#include "drone_class.h"
#include "Sensor_class.h"

typedef struct PID {
	double P,I,D;
	clock_t previousPIDTime; // the last time the PID was updated
	double integratedError;
	double lastError;
	double windupGuard;
}PID;


#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

double updatePID(PID *p, double, double, Drone);

//for range finding/ ESC calibration
void setThrottleRange(void);

int main(void) {

	double euler_array[EULER_DATA_ARRAY];
	for(int i = 0; i < EULER_DATA_ARRAY; i++)
		euler_array[i] = 0.0;

	Drone d;
	Sensor s;

	clock_t pTime = clock();

	//PID yaw; 
	PID *pitch = new struct PID;
	PID *roll = new struct PID;

	pitch->windupGuard = 0.375; //magic numbers
	pitch->P = 0.0;
	pitch->I = 0.0;
	pitch->D = 0.0;
	pitch->previousPIDTime = pTime;
	pitch->integratedError = 0.0;
	pitch->lastError = 0.0;

	roll->windupGuard = 0.375;

	double rollValue = 0.0;
	double pitchValue = 0.0;

	//doubl throttle = 10; // 10%

	while(true){
		s.getData(euler_array);

		printf("Yaw  : %lf \n", euler_array[0]);
		printf("Pitch : %lf \n", euler_array[1]);
		printf("Roll   : %lf \n", euler_array[2]);

		rollValue = updatePID(roll, 3.141593, euler_array[2], d);
		pitchValue = updatePID(pitch, -0.000000, euler_array[1], d);

		usleep(50);

		printf("PID Roll : %lf\n", rollValue);
		printf("PID Pitch : %lf \n\n", pitchValue);

		//updateMotors(rollValue, pitchValue, throttle);
	}


	return 0;
} 

//the magic balance function
//changes current motor speeds, pitch, yaw, roll in accordance with data
//The control algorithm so euler/quaternion values are matched by the motors
double updatePID(PID *p, double target, double current, Drone d){
    clock_t newPIDTime = clock();

    double pTime = (((double)newPIDTime) - ((double)p->previousPIDTime))/CLOCKS_PER_SEC;

    double error = target - current;

    if(d.isLanded()){
        p->integratedError = 0.0;
    } else {
        p->integratedError += error * pTime;
    }

    p->previousPIDTime = constrain(p->integratedError, -(p->windupGuard), p->windupGuard);
    
    double diffTerm = p->D * (current - p->lastError)/(pTime * 100);
    p->lastError = error;
    p->previousPIDTime = newPIDTime;

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



