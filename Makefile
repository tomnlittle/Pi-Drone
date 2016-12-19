CXX		 =g++
CXXFLAGS = -Wall -Werror -std=c++11 -pthread -Ofast
OBJS     = main.o PCA9685.o BNO055_driver-master_src/bno055.o drone_class.o motor_class.o SPI_Handler.o BNO055_Interface.o PID.o
CC=gcc
CFLAGS=-Wall -Werror -std=c11 -g -Ofast
LIBS=-lpthread

main : $(OBJS)
	$(CXX) -o main $(OBJS) $(LIBS)

main.o: main.cpp PCA9685.h defines.h drone_class.h BNO055_Interface.h PID.h

drone_class.o: drone_class.cpp defines.h motor_class.h

motor_class.o: motor_class.cpp PCA9685.h defines.h

PCA9685.o: PCA9685.cpp PCA9685.h

#Sensor_class.o: Sensor_class.cpp Sensor_class.h defines.h SPI_Handler.h BNO055_Interface.h

PID.o : PID.h PID.cpp

SPI_Handler.o: SPI_Handler.cpp SPI_Handler.h

BNO055_Interface.o: BNO055_Interface.cpp BNO055_Interface.h BNO055_driver-master_src/bno055.h

bno055.o: BNO055_driver-master_src/bno055.c BNO055_driver-master_src/bno055.h

clean :
	rm -f main $(OBJS) core



	
