#ifndef __SERIAL_PORT_H__
#define __SERIAL_PORT_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFF_MAXSIZE 1024

int serial_port_open(char *dev);
void serial_port_close(int fd);
int serial_port_set_params(int fd, long speed, char databit, char stopbit,
                           char oebit);
int serial_port_write(int fd, void *buf, int len);
int serial_port_read(int fd, unsigned char *buff, int timeout_ms);

#endif
