#ifndef CREATE2_DRIVER_SERIAL_HPP_
#define CREATE2_DRIVER_SERIAL_HPP_

#include <termios.h>

#define MODEMDEVICE1     "/dev/ttyS0"
#define MODEMDEVICE_USB0 "/dev/ttyUSB0"
#define MODEMDEVICE_USB1 "/dev/ttyUSB1"
#define _POSIX_SOURCE 1
#define FALSE 0
#define TRUE 1

class Serial {
 private:
  int fd_;//, c_, res_;
  struct termios oldtio_, newtio_;
 public:
  Serial(int baudrate, const char* modemdevice, int vmin=0, int lflag=0);
  ~Serial();

  int read(unsigned char* p, int len);
  int write(const unsigned char* p, int len);
  void setVmin(int vmin);  // Minimum #characters to wait at non canonical
  void setRts(int);
};

#endif // CREATE2_DRIVER_SERIAL_HPP_
