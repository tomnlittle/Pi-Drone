#ifndef DEFINES
#include "defines.h"
#endif
#include <fcntl.h> 	
#include <fstream>

#include "BNO055_Interface.h"

class Sensor {
  public:
    // constructor
    Sensor();
    ~Sensor();

    void InitialiseSensors();

    bool getInitialised();

    void getData(double *data_array);

    bool getCalibrated();
    void Calibrate(); 
    void ManualCalibrate(); 
      		
  private:
    bool initialised;
    bool calibrated;

    BNO055_Interface bINT; // Initialises the interface
};