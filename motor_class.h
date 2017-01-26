/*
 * Created on Thu Jan 26 2017 
 *
 * Copyright (c) 2017 Thomas Northall-Little
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
 
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