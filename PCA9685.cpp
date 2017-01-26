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
 
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <syslog.h>		/* Syslog functionallity */
#include <inttypes.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>

#include "PCA9685.h"

void PCA9685::init(int bus, int address) {
	_i2cbus = bus;
	_i2caddr = address;
	snprintf(busfile, sizeof(busfile), "/dev/i2c-%d", bus);
	reset();
}

PCA9685::PCA9685() {}

PCA9685::~PCA9685() {
	reset();
}
//! Sets PCA9685 mode to 00
void PCA9685::reset() {
	openfd();
	write_byte(i2c_file_descriptor, MODE1, 0x00); //Normal mode
	write_byte(i2c_file_descriptor, MODE2, 0x04); //Normal mode
	close(i2c_file_descriptor);
}
//! Set the frequency of PWM
/*!
 \param freq desired frequency. 40Hz to 1000Hz using internal 25MHz oscillator.
 */
void PCA9685::setPWMFreq(int freq) {
	openfd();	
	uint8_t prescale = (CLOCK_FREQ / 4096 / freq)  - 1;
	uint8_t oldmode = read_byte(i2c_file_descriptor, MODE1);
	uint8_t newmode = (oldmode & 0x7F) | 0x10;    //sleep
	write_byte(i2c_file_descriptor, MODE1, newmode);        // go to sleep
	write_byte(i2c_file_descriptor, PRE_SCALER, prescale);
	write_byte(i2c_file_descriptor, MODE1, oldmode);
	usleep(10*1000);
	write_byte(i2c_file_descriptor, MODE1, oldmode | 0x80);
	close(i2c_file_descriptor);	
}

//! PWM a single channel
/*!
 \param led channel to set PWM value for
 \param value 0-4095 value for PWM
 */
void PCA9685::setPWM(uint8_t led, int value) {
	setPWM(led, 0, value);
}
//! PWM a single channel with custom on time
/*!
 \param led channel to set PWM value for
 \param on_value 0-4095 value to turn on the pulse
 \param off_value 0-4095 value to turn off the pulse
 */
void PCA9685::setPWM(uint8_t chan, int on_value, int off_value) {
	openfd();

	write_byte(i2c_file_descriptor, 
						CHANNEL0_ON_L + CHANNEL_MULTIPLYER * chan, on_value & 0xFF);
	write_byte(i2c_file_descriptor, 
						CHANNEL0_ON_H + CHANNEL_MULTIPLYER * chan, on_value >> 8);
	write_byte(i2c_file_descriptor, 
						CHANNEL0_OFF_L + CHANNEL_MULTIPLYER * chan, off_value & 0xFF);
	write_byte(i2c_file_descriptor, 
						CHANNEL0_OFF_H + CHANNEL_MULTIPLYER * chan, off_value >> 8);
	close(i2c_file_descriptor);
}

//! Read a single byte from PCA9685
/*!
 \param fd file descriptor for I/O
 \param address register address to read from
 */
uint8_t PCA9685::read_byte(int fd, uint8_t address) {
	return 0;
	uint8_t buff[BUFFER_SIZE];
	buff[0] = address;
	if (write(fd, buff, BUFFER_SIZE) != BUFFER_SIZE) {
		printf("I2C slave 0x%x failed to go to register 0x%x [read_byte():write %d]", _i2caddr, address, errno);
		return (-1);
	} else {
		if (read(fd, dataBuffer, BUFFER_SIZE) != BUFFER_SIZE) {
			printf ("Could not read from I2C slave 0x%x, register 0x%x [read_byte():read %d]", _i2caddr, address, errno);
			return (-1);
		}
	}
}
//! Write a single byte from PCA9685
/*!
 \param fd file descriptor for I/O
 \param address register address to write to
 \param data 8 bit data to write
 */
void PCA9685::write_byte(int fd, uint8_t address, uint8_t data) {
	uint8_t buff[2];
	buff[0] = address;
	buff[1] = data;
	if (write(fd, buff, sizeof(buff)) != 2) {
		printf("Failed to write to I2C Slave 0x%x @ register 0x%x [write_byte():write %d]", _i2caddr, address, errno);
		usleep(5000);

		//ERROR HANDLING HERE
	}else{
		//printf("Wrote to I2C Slave 0x%x @ register 0x%x [0x%x]\n", _i2caddr, address, data);
	}
}
//! Open device file for PCA9685 I2C bus
/*!
 \return fd returns the file descriptor number or -1 on error
 */
void PCA9685::openfd() {
	int fd = 0;
	if ((fd = open(busfile, O_RDWR)) < 0) {
		printf ("Couldn't open I2C Bus %d [openfd():open %d]", _i2cbus, errno);
		//error handling here
	}
	if (ioctl(fd, I2C_SLAVE, _i2caddr) < 0) {
		printf ("I2C slave %d failed [openfd():ioctl %d]", _i2caddr, errno);
		//Error handling here
	}

	i2c_file_descriptor = fd;
}

