//---------------------------< /-/ AMSL /-/ >------------------------------
/**
 * file         :       serial.cpp
 *
 *
 * Environment  :       g++
 * Latest Update:       2011/05/07
 *
 * Designer(s)  :       y.nishikawa (AMSL)
 * Author(s)    :       y.nishikawa (AMSL)
 *
 * CopyRight    :       2011, Autonomous Mobile Systems Laboratory, Meiji Univ.
 *
 * Revision     :       2011/05/07
 *
 */
//-----------------------------------------------------------------------------

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "roomba_500driver_meiji/serial.h"

#include <iostream>
using namespace std;

//#define DEBUG

Serial::Serial(int baudrate, const char* modemdevice, int vmin, int lflag) {
  struct termios toptions;

  fd_ = open(modemdevice, O_RDWR | O_NOCTTY | O_NDELAY );
  if (fd_ == -1)  {     // Could not open the port.
    perror("roomba_init_serialport: Unable to open port ");
    exit(-1);
  }

  tcgetattr(fd_, &oldtio_);
  if (tcgetattr(fd_, &toptions) < 0) {
    perror("roomba_init_serialport: Couldn't get term attributes");
    exit(-1);
  }

  cfsetispeed(&toptions, baudrate);
  cfsetospeed(&toptions, baudrate);

  // 8N1
  toptions.c_cflag &= ~PARENB;
  toptions.c_cflag &= ~CSTOPB;
  toptions.c_cflag &= ~CSIZE;
  toptions.c_cflag |= CS8;
  // no flow control
  toptions.c_cflag &= ~CRTSCTS;

  toptions.c_cflag    |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
  toptions.c_iflag    &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl

  toptions.c_lflag    &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  toptions.c_oflag    &= ~OPOST; // make raw

  toptions.c_cc[VMIN]  = 26;
  toptions.c_cc[VTIME] = 2; // TODO: FIXME: not sure about this

  if( tcsetattr(fd_, TCSANOW, &toptions) < 0) {
    perror("roomba_init_serialport: Couldn't set term attributes");
    exit(-1);
  }
}


Serial::~Serial() {
  tcsetattr(fd_,TCSANOW,&oldtio_);
  close(fd_);
}


int Serial::read(unsigned char* p, int len) {
  return ::read(fd_, p, len);
}

int Serial::write(const unsigned char* p, int len) {
  return ::write(fd_, p, len);
}

void Serial::setVmin(int vmin) {
  newtio_.c_iflag = IGNPAR;
  newtio_.c_cc[VTIME] = 1;  // Do not use inter-character timer
  newtio_.c_cc[VMIN] = vmin;   //Block until receive this number of charactes
}

void Serial::setRts(int sw) {
  int status;
  ioctl(fd_, TIOCMGET, &status); // set the serial port status

  if(sw) // set the RTS line
    status &= ~TIOCM_RTS;
  else
    status |= TIOCM_RTS;

  ioctl(fd_, TIOCMSET, &status); // set the serial port status
}

#ifdef DEBUG
int main() {
  char rdata[255];
  char sdata[255];
  Serial test(B115200, "/dev/ttyUSB0");

  sprintf(sdata, "V\r");
  test.write_serial(sdata, strlen(sdata));
  test.read_serial(rdata);
  printf("%s",rdata);
  return 0;
}
#endif
