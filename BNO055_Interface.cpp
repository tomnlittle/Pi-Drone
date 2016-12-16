#include "BNO055_Interface.h"

#include <math.h>

struct bno055_t bno055;
SPI_Handler spi;

BNO055_Interface::BNO055_Interface(){
    isCalibrated = false;
    offsetsLoaded = false;

    quaternion_W = BNO055_INIT_VALUE;
	quaternion_X = BNO055_INIT_VALUE;
	quaternion_Y = BNO055_INIT_VALUE;
    quaternion_Z = BNO055_INIT_VALUE;
}

BNO055_Interface::~BNO055_Interface(){}


void BNO055_Interface::InitialiseBNO055(){
    bno055.bus_write = &BNO055_SPI_bus_write; //function pointers for writing to SPI
	bno055.bus_read = &BNO055_SPI_bus_read;
	bno055.delay_msec = &BNO055_delay_msek;

	bno055.dev_addr = 0; //BNO055_ADDRESS; Since we are using SPI to communicate instead of i2c
 
    int32_t result = BNO055_ERROR;
    result = bno055_init(&bno055);

    if(result != 0){
        throw "\nERR: Cannot Initialise BNO055...\n";
    } else {
        cout<< "\nInitialising ";
        cout<< ".";
        bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
        cout<< ".";
        Calibrate();
        cout<< ".";
        CalibrationCheck();
        cout<< " Done \n";
    }
}


void BNO055_Interface::Reset(){
    #if DEBUG
        printf("Resetting BNO055\n");
    #endif
    u8 resetBit = 0x01;
    bno055_set_sys_rst(resetBit);
    sleep(1);
    try {
        InitialiseBNO055();
    } catch (const char* msg){
        sleep(1);
        Reset();
    }
}

void BNO055_Interface::Calibrate(){
    int result = BNO055_ERROR;
    
    result = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);       

    if(offsetsLoaded == false) {
        struct bno055_accel_offset_t accel_data;
        struct bno055_gyro_offset_t gyro_data;
        struct bno055_mag_offset_t mag_data;
        //load file 
        ifstream myfile; 
        myfile.open(CALIBRATION_FILENAME);

        accel_data.x = BNO055_INIT_VALUE;
        accel_data.y = BNO055_INIT_VALUE;
        accel_data.z = BNO055_INIT_VALUE;
        accel_data.r = BNO055_INIT_VALUE;

        gyro_data.x = BNO055_INIT_VALUE;
        gyro_data.y = BNO055_INIT_VALUE;
        gyro_data.z = BNO055_INIT_VALUE;

        mag_data.x = BNO055_INIT_VALUE;
        mag_data.y = BNO055_INIT_VALUE;
        mag_data.z = BNO055_INIT_VALUE;
        mag_data.r = BNO055_INIT_VALUE;        

        myfile >> accel_data.x ;
        myfile >> accel_data.y ;
        myfile >> accel_data.z ;
        myfile >> accel_data.r ;

        myfile >> gyro_data.x ;
        myfile >> gyro_data.y ;
        myfile >> gyro_data.z ;

        myfile >> mag_data.x ;
        myfile >> mag_data.y ;
        myfile >> mag_data.z ;
        myfile >> mag_data.r ;
        
        myfile.close();

        #if DEBUG
            printf("accel x %d \n", accel_data.x);
            printf("accel y %d \n", accel_data.y);
            printf("accel z %d \n", accel_data.z);
            printf("accel r %d \n", accel_data.r);
            printf("gyro x %d \n", gyro_data.x);
            printf("gyro y %d \n", gyro_data.y);
            printf("gyro z %d \n", gyro_data.z);
            printf("mag x %d \n", mag_data.x);
            printf("mag y %d \n", mag_data.y);
            printf("mag z %d \n", mag_data.z);
            printf("mag r %d \n", mag_data.r);
        #endif

        result += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);

        //if successful calibrated = true 
        result += bno055_write_accel_offset(&accel_data);
        result += bno055_write_mag_offset(&mag_data);
        result += bno055_write_gyro_offset(&gyro_data);

        result += bno055_set_operation_mode(DEFAULT_OPERATION_MODE);  

        offsetsLoaded = true;
    } 

    if(result != 0){
        throw "\nERROR: Cannot Calibrate\n";
        return;
    }

    bool checkCalib = isCalibratedSample(10);

    if(checkCalib){
        isCalibrated = true;
    } else {
        printf("\nERR: MANUAL RECALIBRATION IS NECESSARY\n");
    }

    //result += bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);
    //Error handling here
}

void BNO055_Interface::ManualCalibration(){
    int result = BNO055_ERROR;
    u8 accel_calibration = BNO055_INIT_VALUE;
    u8 gyro_calibration = BNO055_INIT_VALUE;
    u8 mag_calibration = BNO055_INIT_VALUE;
    u8 system_calibration = BNO055_INIT_VALUE;

    struct bno055_accel_offset_t accel_data;
    struct bno055_gyro_offset_t gyro_data;
    struct bno055_mag_offset_t mag_data;

    accel_data.x = BNO055_INIT_VALUE;
    accel_data.y = BNO055_INIT_VALUE;
    accel_data.z = BNO055_INIT_VALUE;
    accel_data.r = BNO055_INIT_VALUE;

    gyro_data.x = BNO055_INIT_VALUE;
    gyro_data.y = BNO055_INIT_VALUE;
    gyro_data.z = BNO055_INIT_VALUE;

    mag_data.x = BNO055_INIT_VALUE;
    mag_data.y = BNO055_INIT_VALUE;
    mag_data.z = BNO055_INIT_VALUE;
    mag_data.r = BNO055_INIT_VALUE;

    result = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);           
    result += bno055_set_operation_mode(DEFAULT_OPERATION_MODE);    
    
    printf("Ready to Calibrate\n");

    isCalibrated = false;

    int count = 0;

    while(!isCalibrated){
        result += bno055_get_sys_calib_stat(&system_calibration);
        result += bno055_get_accel_calib_stat(&accel_calibration);
        result += bno055_get_mag_calib_stat(&mag_calibration);
        result += bno055_get_gyro_calib_stat(&gyro_calibration);

        printf("Accel: %d, Gyro: %d, Mag: %d, Sys: %d\n", accel_calibration, 
                                    gyro_calibration, mag_calibration, system_calibration);

        sleep(1);
        if((accel_calibration == BNO055_FULLY_CALIBRATED && mag_calibration == BNO055_FULLY_CALIBRATED) &&
                (gyro_calibration == BNO055_FULLY_CALIBRATED && system_calibration == BNO055_FULLY_CALIBRATED)){

            //this makes sure that the device stays calibrated for 10 seconds
            if(count == 10){
                isCalibrated = true;
            } else {
                count++;
            }
                    
        }
    }

    result += bno055_set_operation_mode(BNO055_OPERATION_MODE_CONFIG);  
    printf("Writing to file  ");  
    //save to file 
    ofstream myfile;                                      
    myfile.open(CALIBRATION_FILENAME, ios::out | ios::trunc);                  
    myfile << accel_data.x << "\n";
    myfile << accel_data.y << "\n";
    myfile << accel_data.z << "\n";
    myfile << accel_data.r << "\n";

    myfile << gyro_data.x << "\n";
    myfile << gyro_data.y << "\n";
    myfile << gyro_data.z << "\n";

    myfile << mag_data.x << "\n";
    myfile << mag_data.y << "\n";
    myfile << mag_data.z << "\n";
    myfile << mag_data.r << "\n";

    myfile.close();                                                 
    printf("\nCalibration Completed \n");   

    result += bno055_set_operation_mode(DEFAULT_OPERATION_MODE);  
    //result += bno055_set_power_mode(BNO055_POWER_MODE_SUSPEND);

    if(result != 0){
        throw "ERROR: Failed to Manually Calibrate";
    }
}

void BNO055_Interface::CalibrationCheck(){
    //if standard offsets have not been loaded then load them
    if(offsetsLoaded != true){
        try{
            Calibrate();
        } catch (const char* msg){
            cout << msg << endl;
        } 
    }
        
    bool checkCalib = isCalibratedSample(10);

    if(checkCalib){
        isCalibrated = true;
    } else {
        isCalibrated = false;

        try{
            Calibrate();
        } catch (const char* msg){
            cout << msg << endl;
        }
    } 
}

bool BNO055_Interface::isCalibratedSample(int sampleLength){
    //not actually check if the device is calibrated
    int result = BNO055_ERROR;
    u8 accel_calibration = BNO055_INIT_VALUE;
    u8 gyro_calibration = BNO055_INIT_VALUE;
    u8 mag_calibration = BNO055_INIT_VALUE;

    double sampleAverage = 0;
    
    result = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    result += bno055_set_operation_mode(DEFAULT_OPERATION_MODE);  
   // result += bno055_get_sys_calib_stat(&system_calibration);
    result += bno055_get_accel_calib_stat(&accel_calibration);
    result += bno055_get_mag_calib_stat(&mag_calibration);
    result += bno055_get_gyro_calib_stat(&gyro_calibration);

    for(int i = 0; i < sampleLength; i++){
        if((accel_calibration == BNO055_FULLY_CALIBRATED /*&& mag_calibration == BNO055_FULLY_CALIBRATED*/) &&
                (gyro_calibration == BNO055_FULLY_CALIBRATED)){
                    sampleAverage += 1;
        }
        usleep(100);
    }

    //if more of the samples are positive according to this threshold then return true
    if(sampleAverage/sampleLength >= CALIB_SAMPLE_THRESHOLD){
        return true;
    } else {
        return false;
    }
}

bool BNO055_Interface::getCalibrated(){
    return isCalibrated;
}

void BNO055_Interface::getEulerData(double *data_array){
    try {
        updateData();
    } catch (const char* msg){
        cout << msg << endl;
        return;
    }
    
    double sqw = quaternion_W*quaternion_W;
    double sqx = quaternion_X*quaternion_X;
    double sqy = quaternion_Y*quaternion_Y;
    double sqz = quaternion_Z*quaternion_Z;

    data_array[0] = atan2(2.0*(quaternion_X*quaternion_Y+quaternion_Z*quaternion_W),(sqx-sqy-sqz+sqw));//X - Yaw
    data_array[1] = asin(-2.0*(quaternion_X*quaternion_Z-quaternion_Y*quaternion_W)/(sqx+sqy+sqz+sqw)); // Y - Pitch 
    data_array[2] = atan2(2.0*(quaternion_Y*quaternion_Z+quaternion_X*quaternion_W),(-sqx-sqy+sqz+sqw)); // Z - Roll
}

void BNO055_Interface::updateData(){
    int result = BNO055_ERROR;

    /* set the power mode as NORMAL*/
    //result = bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
    result = bno055_set_operation_mode(DEFAULT_OPERATION_MODE);

    result += bno055_read_quaternion_w(&quaternion_W);
	result += bno055_read_quaternion_x(&quaternion_X);
	result += bno055_read_quaternion_y(&quaternion_Y);
	result += bno055_read_quaternion_z(&quaternion_Z);

    quaternion_W = quaternion_W/BNO055_RADIAN_CONSTANT;
    quaternion_X = quaternion_X/BNO055_RADIAN_CONSTANT;
    quaternion_Y = quaternion_Y/BNO055_RADIAN_CONSTANT;
    quaternion_Z = quaternion_Z/BNO055_RADIAN_CONSTANT;
    
    //Finished with read 
    if(result != 0){
        throw "ERROR: Failed to Update Data";
    }
}

s8 BNO055_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    //get the file descriptor here ???
    spi.openfd();
    int file_descriptor = spi.getFileDescriptor();

    #if DEBUG
        printf("\n\nSPI READ\n");
        printf("reg_addr %d \n", reg_addr);
        printf("reg_data ");
        printf("size of %d\n", sizeof(reg_data));
        for(uint i = 0; i < cnt; i++){
            printf("%d - ", reg_data[i]);
        }
        printf("\n");
        printf("cnt %d \n", cnt);
    #endif

    //clear output buffer
    tcflush(file_descriptor, TCOFLUSH);
    //flush input buffer
    tcflush(file_descriptor, TCIFLUSH);

    u8 result = BNO055_ERROR;
    u8 array[4];

    //ask to read data
    array[0] = 0xAA;
    array[1] = 0x01;
    array[2] = reg_addr & 0xFF;
    array[3] = cnt & 0xFF;

    #if DEBUG
        printf("Writing the data \n");
    #endif
    //write get input command
    result = write(file_descriptor, array, 4);

    //wait until data has been sent
   // tcdrain(file_descriptor);
    #if DEBUG
        printf("WAITING FOR RESPONSE\n");
    #endif

    //get response
    uint response_length = cnt + 2;
    u8 resp_array[response_length];    
    uint bytes = 0;
    uint loop_break = SERIAL_LOOP_BREAK;
    while(bytes < response_length && loop_break > 0){
        ioctl(file_descriptor, FIONREAD, &bytes);
        BNO055_delay_msek(10);
        #if DEBUG
            printf(".");
        #endif
        loop_break--;
    }
    #if DEBUG
        printf("\nREADING DATA ");
    #endif
    result = read(file_descriptor, resp_array, response_length);

    if(resp_array[0] != 0xBB){ 
        #if DEBUG
            printf("... ERROR\n");
        #endif
        return BNO055_ERROR;
    }

    for(uint i = 0; i < cnt; i++){
        reg_data[i] = resp_array[i + 2];
    }
    #if DEBUG
        printf("... DONE\n");
    #endif

    //close the file descriptor here ???
    spi.closefd();

    result = 0;
    return result;
}

s8 BNO055_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    //open the file descriptor ???
    spi.openfd();
    int file_descriptor = spi.getFileDescriptor();

    #if DEBUG
        printf("WRITING\n");
        printf("reg_addr %d \n", reg_addr);
        printf("reg_data ");
        printf("size of %d\n", sizeof(reg_data));
        for(uint i = 0; i < cnt; i++){
            printf("%d - ", reg_data[i]);
        }
        printf("\n");
        printf("cnt %d \n", cnt);  
    #endif

    //clear output buffer
    tcflush(file_descriptor, TCOFLUSH);
    //flush input buffer
    tcflush(file_descriptor, TCIFLUSH);
    s32 result = BNO055_INIT_VALUE;

    //write to file
    uint array_length = 4 + cnt;
    u8 array[array_length];
    array[0] = 0xAA;
    array[1] = 0x00;    //initiates write
    array[2] = reg_addr & 0xFF;     //writes reg address
    array[3] = cnt;

    //write data to array
    for(uint i = 0; i < cnt; i++){
        array[4 + i] = reg_data[i];
    }

    #if DEBUG
        printf("Array Values => ");
        for(uint i = 0; i < array_length; i++){
            printf("0x%02x ", array[i]);
        }
        printf("\n");
    #endif

    //write array to output buffer
    result = write(file_descriptor, array, array_length);

    //hold system until data has been transmitted
    //tcdrain(file_descriptor);

    //wait for response
    uint response_length = 2;
    //return error if response is invalid
    uint bytes = 0;
    uint loop_break = SERIAL_LOOP_BREAK;
    while(bytes < response_length && loop_break > 0){
        ioctl(file_descriptor, FIONREAD, &bytes);
        BNO055_delay_msek(10);
        #if DEBUG
            printf(".");
        #endif
        loop_break--;
    }

    u8 resp_array[response_length];
    result = read(file_descriptor, resp_array, response_length);

    #if DEBUG
        printf("Bytes %d\n", bytes);
        for(uint i = 0; i < bytes; i++)
                printf("0x%02x ", resp_array[i]);
        printf("\n0 Value %d \n", resp_array[0]);
    #endif

    if(resp_array[0] != 0xEE && resp_array[1] != 0x01){
        BNO055_delay_msek(5000);
        return BNO055_ERROR;
    }

    //close the file descriptor here ?
    spi.closefd();
    result = 0;
	return result;  
}

void BNO055_delay_msek(u32 msek)
{
    usleep(SERIAL_WAIT * msek);
}

 