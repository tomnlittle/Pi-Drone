CXX		 =g++
CXXFLAGS = -Wall -Werror
OBJS     = main.o PCA9685.o bno055.o drone_class.o Sensor_class.o motor_class.o SPI_Handler.o BNO055_Interface.o
CC=gcc
CFLAGS=-Wall -Werror -g

main : $(OBJS)
	$(CXX) -o main $(OBJS)

main.o: main.cpp PCA9685.h defines.h drone_class.h Sensor_class.h

drone_class.o: drone_class.cpp defines.h motor_class.h

motor_class.o: motor_class.cpp PCA9685.h defines.h

PCA9685.o: PCA9685.cpp PCA9685.h

Sensor_class.o: Sensor_class.cpp Sensor_class.h defines.h SPI_Handler.h BNO055_Interface.h

SPI_Handler.o: SPI_Handler.cpp SPI_Handler.h

BNO055_Interface.o: BNO055_Interface.cpp BNO055_Interface.h bno055.h

bno055.o: bno055.c bno055.h

clean :
	rm -f main $(OBJS) core



	