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

#include <fcntl.h> 
#include <sys/ioctl.h>
#include <cerrno>
#include <string.h>		
#include <fstream>

#include "SPI_Handler.h"

extern "C" {
  //this tells the compiler that this is a header for c code
  #include "BNO055_driver-master_src/bno055.h"
}

#define BNO055_DEGREE_CONSTANT                  16.00
#define BNO055_RADIAN_CONSTANT                  900.00

#define BNO055_FULLY_CALIBRATED                 3
#define CALIBRATION_FILENAME                    "./BNO055_CALIBRATION_DATA.txt"
#define CALIB_SAMPLE_THRESHOLD                  0.4
#define DEFAULT_OPERATION_MODE                 BNO055_OPERATION_MODE_NDOF
//BNO055_OPERATION_MODE_IMUPLUS//BNO055_OPERATION_MODE_NDOF <- these appear to very inaccurate

 /*----------------------------------------------------------------------------*
 *  struct bno055_t parameters can be accessed by using BNO055
 *	BNO055_t having the following parameters
 *	Bus write function pointer: BNO055_WR_FUNC_PTR
 *	Bus read function pointer: BNO055_RD_FUNC_PTR
 *	Burst read function pointer: BNO055_BRD_FUNC_PTR
 *	Delay function pointer: delay_msec
 *	I2C address: dev_addr
 *	Chip id of the sensor: chip_id
*---------------------------------------------------------------------------*/

//Read and write Library for SPI 
s8 BNO055_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
s8 BNO055_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_delay_msek(u32 msek);

class BNO055_Interface {
    public:
        BNO055_Interface();
        ~BNO055_Interface();

        void InitialiseBNO055();
        void Reset();

        //writes the euler/quaterion data to the array
        void Calibrate();
        void ManualCalibration();
        bool getCalibrated();

        double getYaw();
        double getRoll();
        double getPitch();

    private:
        void updateData();
        //void CalibrationCheck();
        bool isCalibratedSample(int sampleLength);
        void writeEulerData();

        bool isCalibrated;
        /*
        determines whether the offsets for calibration have already been loaded
        if the offsets have been loaded and the CalibrationCheck function returns false 
        a full manual recalibration will be required as the devices offset values need to be changed 
        */
        bool offsetsLoaded; 

        double yaw;
        double roll;
        double pitch;

        bool threadActive;
        std::thread updateThread;  //the thread for updating the data 

        int numUpdates;
};