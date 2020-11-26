// https://stackoverflow.com/questions/49577244/serial-communication-between-rpi-and-arduino-using-c

#include <stdio.h>
#include <string.h>
#include <errno.h>

#include <wiringSerial.h>

int main ()
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
}