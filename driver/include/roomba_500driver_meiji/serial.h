//---------------------------< /-/ AMSL /-/ >------------------------------
/**
 * file         :       serial.h
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

#ifndef _SERIAL_H
#define _SERIAL_H

#include <termios.h>

#define MODEMDEVICE1 "/dev/ttyS0"
#define MODEMDEVICE_USB0 "/dev/ttyUSB0"
#define MODEMDEVICE_USB1 "/dev/ttyUSB1"
#define _POSIX_SOURCE 1 /* POSIX 準拠のソース */
#define FALSE 0
#define TRUE 1

//#define DEBUG

class Serial
{
private:

	int fd_;//, c_, res_;
	struct termios oldtio_, newtio_;


public:

	Serial(int baudrate, const char* modemdevice, int vmin=0, int lflag=0);
	~Serial();

	int read(unsigned char* p, int len);
	int write(const unsigned char* p, int len);
	void setVmin(int vmin);  // non canonical 時のreadで待つ最低限の文字数
	void setRts(int);

};

#endif //_SERIAL_H
