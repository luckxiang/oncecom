#include "serial_port.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

int main(int argc, char *argv[])
{
  int fd;
  int i = 0;
  unsigned char line[BUFF_MAXSIZE] = {0};
  struct timeval start;
  struct timeval end;
  int time_use_ms;

  fd = serial_port_open("/dev/ttyUSB0");
  serial_port_set_params(fd, 115200, 8, 1, 'N');

  for (i = 1; i < argc; i++) {
    if (1 == i)
      sprintf(line, "%s", argv[i]);
    else
      sprintf(line, "%s %s", line, argv[i]);
  }

  sprintf(line, "%s\n", line);
  printf(">%s", line);
  serial_port_write(fd, line, strlen(line));
  printf("<");
  do {
    gettimeofday(&start, NULL);
    memset(line, 0, BUFF_MAXSIZE);
    serial_port_read(fd, line, 100);
    gettimeofday(&end, NULL);
    time_use_ms = ((end.tv_sec - start.tv_sec) * 1000000 +
                   (end.tv_usec - start.tv_usec)) /
                  1000;
    if (time_use_ms >= 100) {
      break;
    }
    printf("%s", line);

  } while (1);
  printf("\n");

  serial_port_close(fd);

  return 0;
}
