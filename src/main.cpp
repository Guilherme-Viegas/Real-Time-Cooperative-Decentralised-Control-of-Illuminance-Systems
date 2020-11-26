// https://stackoverflow.com/questions/49577244/serial-communication-between-rpi-and-arduino-using-c

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <iostream>
#include <wiringSerial.h>
/*
int main(int argc, char *argv[])
{
  int fd ;

  if((fd=serialOpen("/dev/ttyACM0",2000000))<0){
    fprintf(stderr,"Unable to open serial device: %s\n",strerror(errno));
    return 0;
  }

  for (;;){
    putchar(serialGetchar(fd));
    fflush(stdout);
  }
}*/

int main(int argc, char** argv) {
    std::cout << "Have " << argc << " arguments:" << std::endl;
    for (int i = 0; i < argc; ++i) {
        std::cout << argv[i] << std::endl;
    }

    int fd ;

  if((fd=serialOpen("/dev/ttyACM0",2000000))<0){
    fprintf(stderr,"Unable to open serial device: %s\n",strerror(errno));
    return 0;
  }

  for (;;){
    //putchar(serialGetchar(fd));
    //fflush(stdout);
  }
}