#ifndef _PCA9685_H
#define _PCA9685_H
#include <inttypes.h>

// Register Definitions

#define MODE1 					0x00			//Mode  register  1
#define MODE2 					0x01			//Mode  register  2
#define SUBADDR1 				0x02			//I2C-bus subaddress 1
#define SUBADDR2 				0x03			//I2C-bus subaddress 2
#define SUBADDR3 				0x04			//I2C-bus subaddress 3
#define ALLCALLADDR				0x05     		//LED All Call I2C-bus address
#define CHANNEL0				0x06			//LED0 start register
#define CHANNEL0_ON_L 			0x06			//LED0 output and brightness control byte 0
#define CHANNEL0_ON_H 			0x07			//LED0 output and brightness control byte 1
#define CHANNEL0_OFF_L 			0x08			//LED0 output and brightness control byte 2
#define CHANNEL0_OFF_H 			0x09			//LED0 output and brightness control byte 3
#define CHANNEL_MULTIPLYER 		4				// For the other 15 channels
#define ALL_CHANNEL_ON_L 		0xFA   			//load all the LEDn_ON registers, byte 0 (turn 0-7 channels on)
#define ALL_CHANNEL_ON_H 		0xFB			//load all the LEDn_ON registers, byte 1 (turn 8-15 channels on)
#define ALL_CHANNEL_OFF_L		0xFC			//load all the LEDn_OFF registers, byte 0 (turn 0-7 channels off)
#define ALL_CHANNEL_OFF_H		0xFD			//load all the LEDn_OFF registers, byte 1 (turn 8-15 channels off)
#define PRE_SCALER 				0xFE			//prescaler for output frequency
#define CLOCK_FREQ 				25000000.0 		//25MHz default osc clock
#define BUFFER_SIZE 			0x08  			//1 byte buffer
#define MAX_BUFFER_SIZE			64				//Max size of the busfile

//! Main class that exports features for PCA9685 chip
class PCA9685 {
public:
	PCA9685();
	void init(int,int);
	virtual ~PCA9685();
	void reset(void);
	void setPWMFreq(int);
	void setPWM(uint8_t, int, int);
	void setPWM(uint8_t, int);
private:
	int _i2caddr;
	int _i2cbus;
	char busfile[MAX_BUFFER_SIZE];
	int i2c_file_descriptor; //tempoarily holds the file descritpor being used
	uint8_t dataBuffer[BUFFER_SIZE];
	uint8_t read_byte(int, uint8_t);
	void write_byte(int, uint8_t, uint8_t);
	void openfd(); // called multiple, stores the file descriptor
};
#endif


