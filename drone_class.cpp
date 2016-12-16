#include "drone_class.h"

Drone::Drone(){
    avg_speed = 0;
    landed = true;

    PCA9685 new_PWM;
	new_PWM.init(I2C_ADAPTER, PCA9685_ADDRESS);
	new_PWM.setPWMFreq(PCA9685_FREQUENCY);
    pwm = new_PWM;

    //Initialise the motors
    Front_Left.init(MOTOR_0_PIN, pwm);
    Front_Right.init(MOTOR_1_PIN, pwm);
    Back_Left.init(MOTOR_2_PIN, pwm);
    Back_Right.init(MOTOR_3_PIN, pwm);

    Front_Left.setSpeed(0);
    Front_Right.setSpeed(0);
    Back_Left.setSpeed(0);
    Back_Right.setSpeed(0);

	sleep(5);
}

Drone::~Drone(){
    Front_Left.setSpeed(0);
    Front_Right.setSpeed(0);
    Back_Left.setSpeed(0);
    Back_Right.setSpeed(0);
}

bool Drone::isLanded(){
    return landed;
}

void Drone::setLanded(bool value){
    landed = value;
}

int Drone::getAvgSpeed(){
    return avg_speed;
}

void Drone::updateMotors(double roll, double pitch, double yaw, int throttle){
    Front_Left.setSpeed(0);
    Front_Right.setSpeed(0);
    Back_Left.setSpeed(0);
    Back_Right.setSpeed(0);
}


/*

void applyMotorCommand() {
  motorCommand[FRONT_LEFT]  = throttle - motorAxisCommandPitch + motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[FRONT_RIGHT] = throttle - motorAxisCommandPitch - motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_LEFT]   = throttle + motorAxisCommandPitch + motorAxisCommandRoll + (YAW_DIRECTION * motorAxisCommandYaw);
  motorCommand[REAR_RIGHT]  = throttle + motorAxisCommandPitch - motorAxisCommandRoll - (YAW_DIRECTION * motorAxisCommandYaw);
}






	printf ("Servo Initialising\n");
	
	printf("...");
	d->pwm.init(PCA9685_I2C_ADAPTER, PCA9685_ADDRESS);
	printf("Initialised...\n");
	sleep(1);
	
	printf ("Setting frequency..\n");
	d->pwm.setPWMFreq(PCA9685_FREQUENCY);
	printf("Frequency Set\n");
	sleep(1);

	printf("Initialising Motors...\n");
	printf("HOLDING THROTTLE IN DOWN POSITION... waiting... \n");
	set_motor_speed(d->pwm, 0, MOTOR_0);
	set_motor_speed(d->pwm, 0, MOTOR_1);
	set_motor_speed(d->pwm, 0, MOTOR_2);
	set_motor_speed(d->pwm, 0, MOTOR_3);
	sleep(5);

    */