#include "Sensor_class.h"


Sensor::Sensor(){
    initialised = false;
    calibrated = false;
    InitialiseSensors();
}

Sensor::~Sensor(){}

void Sensor::InitialiseSensors(){
    try {
        bINT.InitialiseBNO055();
    } catch (const char* msg){
        cout << msg << endl;
        sleep(2);
        bINT.Reset();
    }
    
    if(bINT.getCalibrated() == true){
        calibrated = true;
    }

    initialised = true;
}

void Sensor::getData(double *data_array){
    bINT.getEulerData(data_array);
}

void Sensor::Calibrate(){
    try{
        bINT.Calibrate();
    } catch (const char* msg){
        cout << msg << endl;
    }
   
}

void Sensor::ManualCalibrate(){
    printf("\n\nENTERING MANUAL CALIBRATION MODE \n");
    try{
        bINT.ManualCalibration();
    } catch (const char* msg){
        cout << msg << endl;
    }
} 

bool Sensor::getCalibrated(){
    if(bINT.getCalibrated())
        calibrated = true;

    return calibrated;
}

bool Sensor::getInitialised(){
    return initialised;
}


