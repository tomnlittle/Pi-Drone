#include "SPI_Handler.h"
#include <errno.h>

SPI_Handler::SPI_Handler(){
    
    serial_file_descriptor = open(SERIAL_PORT_0, O_RDWR);
/*
     if (serial_file_descriptor == -1){
        //if serial 0 fails try the second one
        printf ("Error no is : %d\n", errno);
        //printf("Error description is : %s\n",strerror(errno));
        sleep(1); // sleep for 1 second and then try the second file
        serial_file_descriptor = open(SERIAL_1, O_RDWR);
        serial_port = SERIAL_1;
    };
*/
    if (serial_file_descriptor == -1){
        printf ("Error no is : %d\n", errno);
        //printf("Error description is : %s\n",strerror(errno));
    };

    //struct termios options;
    tcgetattr(serial_file_descriptor, &options);
    
    //configure settings for SPI
    cfsetospeed(&options, SERIAL_BAUD); //for output speed    
    cfsetispeed(&options, SERIAL_BAUD);  //for input speed

    //set data bits
    options.c_cflag &= ~CSIZE; 
    options.c_cflag |= SERIAL_DATA_BITS;

    options.c_cflag |= SERIAL_PARITY;

    //set one stop bit
    options.c_cflag &=~ CSTOPB;

    options.c_cflag |= CLOCAL;
    options.c_cflag |= CREAD;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    options.c_cc[VTIME] = SERIAL_MIN_TIME;
    options.c_cc[VMIN] = SERIAL_MIN_LENGTH;


    tcsetattr(serial_file_descriptor, TCSANOW, &options);

    #if DEBUG
        cout<< "SPI DONE \n";
    #endif

    closefd();
}

SPI_Handler::~SPI_Handler(){
    closefd();
}

void SPI_Handler::openfd(){
    //std::ifstream file (SERIAL_PORT_0);
    //file.exceptions ( std::ifstream::failbit | std::ifstream::badbit );

    serial_file_descriptor = open(SERIAL_PORT_0, O_RDWR);
    isOpen = true;
    serialFlush();
}

void SPI_Handler::closefd(){
    isOpen = false;
    close(serial_file_descriptor);
}

void SPI_Handler::serialFlush(){
    tcflush(serial_file_descriptor, TCOFLUSH);
    tcflush(serial_file_descriptor, TCIFLUSH);
}

void SPI_Handler::serialWrite(uint8_t *data, int size){
    if(!isOpen){
        openfd();
    }

    write(serial_file_descriptor, data, size);
    printf(".");
    tcdrain(serial_file_descriptor);
}

void SPI_Handler::serialRead(uint8_t *data, int size){
    if(!isOpen){
        openfd();
    }
    
    int bytes = 0;
    int loop_break = SERIAL_LOOP_BREAK;
    while(bytes < size && loop_break > 0){
        ioctl(serial_file_descriptor, FIONREAD, &bytes);
        usleep(SERIAL_WAIT * 10);
        loop_break--;
    }
    
    read(serial_file_descriptor, data, size);
}
