#include "BNO055_Interface.h"

#include <math.h>

struct bno055_t bno055;
SPI_Handler spi;
struct bno055_quaternion_t quaternion_data; //struct for data 
struct bno055_quaternion_t quaternion_data_old; //struct for data 

BNO055_Interface::BNO055_Interface(){
    isCalibrated = false;
    offsetsLoaded = false;

    quaternion_data.w = BNO055_INIT_VALUE;
    quaternion_data.x = BNO055_INIT_VALUE;
    quaternion_data.y = BNO055_INIT_VALUE;
    quaternion_data.z = BNO055_INIT_VALUE;

    yaw = 0.0;
    pitch = 0.0;
    roll = 0.0;

    numUpdates = 0;
}

BNO055_Interface::~BNO055_Interface(){
    threadActive = false;
    spi.closefd();
    if(updateThread.joinable()) updateThread.join();
    cout << "BNO055 Destructor : Number of Updates " << numUpdates << "\n";
}


void BNO055_Interface::InitialiseBNO055(){
    cout<< "\nInitialising ";
    bno055.bus_write = &BNO055_SPI_bus_write; //function pointers for writing to SPI
	bno055.bus_read = &BNO055_SPI_bus_read;
	bno055.delay_msec = &BNO055_delay_msek;

	bno055.dev_addr = 0; //BNO055_ADDRESS; Since we are using SPI to communicate instead of i2c

    //clears all the data in the registers, this is important if the system is restarting 
    //after a soft reset as it forces a hard reset
    u8 resetBit = 0x01;
    bno055_set_sys_rst(resetBit);
    usleep(635);
 
    int32_t result = BNO055_ERROR;
    result = bno055_init(&bno055);

    if(result != 0){
        throw "\nERR: Cannot Initialise BNO055...\n";
    } else {
        cout<< ".";
        bno055_set_power_mode(BNO055_POWER_MODE_NORMAL);
        cout<< ".";
        Calibrate();
        cout<< " Done ... Starting Thread BNO055 Thread\n";

        bno055_set_operation_mode(DEFAULT_OPERATION_MODE);
        updateThread = std::thread(&BNO055_Interface::writeEulerData, this);
        threadActive = true;
    }
}

void BNO055_Interface::Reset(){
    printf("Resetting BNO055\n");
    try {
        if(updateThread.joinable()) updateThread.join();
        threadActive = false;
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

    if(result == BNO055_ERROR){
        throw "ERROR: Failed to Manually Calibrate";
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
            sampleAverage += 1.00;
        } 
        result += bno055_get_accel_calib_stat(&accel_calibration);
        result += bno055_get_gyro_calib_stat(&gyro_calibration);
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

void BNO055_Interface::writeEulerData(){
    while(threadActive){
        try {
            updateData();
        } catch (const char* msg){
            cout << msg << endl;
            continue;
        }

        s16 quaternion_W = quaternion_data.w/BNO055_RADIAN_CONSTANT;
        s16 quaternion_X = quaternion_data.x/BNO055_RADIAN_CONSTANT;
        s16 quaternion_Y = quaternion_data.y/BNO055_RADIAN_CONSTANT;
        s16 quaternion_Z = quaternion_data.z/BNO055_RADIAN_CONSTANT;
        
        double sqw = quaternion_W*quaternion_W;
        double sqx = quaternion_X*quaternion_X;
        double sqy = quaternion_Y*quaternion_Y;
        double sqz = quaternion_Z*quaternion_Z;

        yaw = atan2(2.0*(quaternion_X*quaternion_Y+quaternion_Z*quaternion_W),(sqx-sqy-sqz+sqw));//X - Yaw
        pitch = asin(-2.0*(quaternion_X*quaternion_Z-quaternion_Y*quaternion_W)/(sqx+sqy+sqz+sqw)); // Y - Pitch 
        roll = atan2(2.0*(quaternion_Y*quaternion_Z+quaternion_X*quaternion_W),(-sqx-sqy+sqz+sqw)); // Z - Roll

        if(roll < 0){
            double diff =  3.141593 + roll;
            roll = diff + 3.141593;
        }
    }
}

int checkThreshold(s16 old_value, s16 new_value){
	//if((old_value + new_value)/2 > (old_value + 5.00)){
	if((old_value + 5) < new_value){
		return BNO055_ERROR;
	}
	return 0;
}

void BNO055_Interface::updateData(){
    int result = BNO055_ERROR;
    // result = bno055_set_operation_mode(DEFAULT_OPERATION_MODE);

    quaternion_data_old = quaternion_data;
    result = bno055_read_quaternion_wxyz(&quaternion_data);

    //check data integrity
    result += checkThreshold(quaternion_data_old.w, quaternion_data.w);
    result += checkThreshold(quaternion_data_old.x, quaternion_data.x);
    result += checkThreshold(quaternion_data_old.y, quaternion_data.y);
    result += checkThreshold(quaternion_data_old.z, quaternion_data.z);

    //Finished with read 
    if(result != 0){
        throw "ERROR: Failed to Retrieve Data";
    } else {
        //printf("update successful\n");
        numUpdates++;
    }
}

double BNO055_Interface::getYaw(){
    return yaw;
}

double BNO055_Interface::getRoll(){
    return roll;
}

double BNO055_Interface::getPitch(){
    return pitch;
}

s8 BNO055_SPI_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt){
    //get the file descriptor here ???
    spi.openfd();

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

    u8 result = BNO055_ERROR;
    u8 array[4];

    //ask to read data
    array[0] = 0xAA;
    array[1] = 0x01;
    array[2] = reg_addr & 0xFF;
    array[3] = cnt & 0xFF;

    //write get input command
    spi.serialWrite(array, 4);

    //wait until data has been sent
   // tcdrain(file_descriptor);

    uint response_length = cnt + 2;
    u8 resp_array[response_length];    
    spi.serialRead(resp_array, response_length);

    if(resp_array[0] != 0xBB){ 
        #if DEBUG
            printf("... ERROR\n");
        #endif
        return BNO055_ERROR;
    }

    for(uint i = 0; i < cnt; i++){
        reg_data[i] = resp_array[i + 2];
    }

    //close the file descriptor here ???
    spi.closefd();

    result = 0;
    return result;
}

s8 BNO055_SPI_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt){
    //open the file descriptor ???
    spi.openfd();

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
    
    u8 result = BNO055_INIT_VALUE;

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

    //write array to output buffer
    spi.serialWrite(array, array_length);

    //wait for response
    uint response_length = 2;

    u8 resp_array[response_length];
    spi.serialRead(resp_array, response_length);

    if(resp_array[0] != 0xEE && resp_array[1] != 0x01){
        BNO055_delay_msek(5000);
        return BNO055_ERROR;
    }

    //close the file descriptor here ?
    spi.closefd();
    result = 0;
	return result;  
}

void BNO055_delay_msek(u32 msek){
    usleep(SERIAL_WAIT * msek);
}