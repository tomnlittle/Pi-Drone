#ifndef DEFINES
        #include "defines.h"
#endif

#ifndef PCA9685
      #include "PCA9685.h"
#endif

#include "motor_class.h"


class Drone {
        public:
                // constructor
                Drone();
                ~Drone();
                bool isLanded();
                void setLanded(bool);
                int getAvgSpeed();
                void updateMotors(double, double, double, double);
        private:
                PCA9685 pwm;
                int avg_speed;
                bool landed;

                Motor Front_Left;
                Motor Front_Right;
                Motor Back_Left;
                Motor Back_Right;
};