
#ifndef DEFINES
      #include "defines.h"
#endif


#ifndef PCA9685
      #include "PCA9685.h"
#endif

/*
Motor Positions 
G A is where the Gyro/Accelerometer is position 

				CW	0	      1     CCW
					 \         /
					  \  G A  /
					   \	   /
					      *
						*
					      *
						*
					   /     \
					  /       \
					 /         \
			      CCW   2		3     CW
*/

class Motor {
   public:
      // constructor
      void init(int motor_pin, PCA9685 pwm);

      Motor();
      ~Motor();
		
      int getSpeed();
      double getDutyCycle();
      void setSpeed(double);

   private:
      int pin; // Pin Number on the PCA9685
      int speed; //expressed as a percentage between 0 and 100. IT IS AN INT because they are nicer to work with than floats
      double raw_speed; //Actual dutyCycle
      PCA9685 pwm;
};