#include "serial_port.h"
#include <errno.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

int serial_port_open(char *dev)
{
  int fd = -1;
  
  fd = open(dev, O_RDWR | O_NOCTTY | O_NDELAY);
  if (-1 == fd)
    printf("Can't Open Serial Port!\n");
    
  return fd;
}

void serial_port_close(int fd)
{
  close(fd);
}

int serial_port_set_params(int fd, long speed, char databit, char stopbit,
                           char oebit)
{
  struct termios opt;
  int err = -1;

  tcgetattr(fd, &opt);

  switch (databit) {
  case 7:
    opt.c_cflag |= CS7;
    break;
  case 8:
    opt.c_cflag |= CS8;
    break;
  default:
    opt.c_cflag |= CS8;
    break;
  }

  switch (stopbit) {
  case 1:
    opt.c_cflag &= ~CSTOPB;
    break;
  case 2:
    opt.c_cflag |= CSTOPB;
    break;
  default:
    opt.c_cflag &= ~CSTOPB;
    break;
  }

  switch (oebit) {
  case 'O': // odd
    opt.c_cflag |= PARENB;
    opt.c_cflag |= (INPCK | ISTRIP);
    opt.c_cflag |= PARODD;
    break;
  case 'E': // even
    opt.c_cflag |= PARENB;
    opt.c_cflag |= (INPCK | ISTRIP);
    opt.c_cflag &= ~PARODD;
    break;
  case 'N':
    opt.c_cflag &= ~PARODD;
    break;
  default:
    opt.c_cflag &= ~PARODD;
    break;
  }

  switch (speed) {
  case 2400:
    cfsetispeed(&opt, B2400);
    cfsetospeed(&opt, B2400);
    break;
  case 4800:
    cfsetispeed(&opt, B4800);
    cfsetospeed(&opt, B4800);
    break;
  case 9600:
    cfsetispeed(&opt, B9600);
    cfsetospeed(&opt, B9600);
    break;
  case 57600:
    cfsetispeed(&opt, B57600);
    cfsetospeed(&opt, B57600);
    break;
  case 115200:
    cfsetispeed(&opt, B115200);
    cfsetospeed(&opt, B115200);
    break;
  default:
    cfsetispeed(&opt, B9600);
    cfsetospeed(&opt, B9600);
    break;
  }

  opt.c_cc[VTIME] = 0;
  opt.c_cc[VMIN] = 0;
  opt.c_cflag |= CLOCAL | CREAD;
  opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  tcflush(fd, TCIFLUSH);

  err = tcsetattr(fd, TCSANOW, &opt);

  return err;
}

int serial_port_write(int fd, void *buf, int len)
{
  int m_fd = fd;
  int write_count = 0;
  int nwrite = 0;

  if (m_fd < 0) {
    printf("Please Open The Device File!\n");
  }

  while (len > 0) {
    nwrite = write(fd, (unsigned char *)buf + write_count, len);
    if (nwrite < 1) {
      printf("Write Datda Fail!\n");
      break;
    }
    write_count += nwrite;
    len -= nwrite;
  }

  tcflush(fd, TCIOFLUSH);
  return write_count;
}

int serial_port_read(int fd, unsigned char *buffer, int timeout_ms)
{
  int nread = 0;
  int fd_max;
  int nselect;
  struct timeval tv;

  fd_set readfds;
  FD_ZERO(&readfds);
  FD_SET(fd, &readfds);
  fd_max = fd + 1;

  tv.tv_sec = 0;
  tv.tv_usec = timeout_ms * 1000;

  nselect = select(fd_max, &readfds, NULL, NULL, &tv);
  memset(buffer, 0, sizeof(buffer));

  if (FD_ISSET(fd, &readfds) > 0) {
    nread = read(fd, buffer, 8);
    buffer[nread] = '\0';
  }

  return nread;
}
