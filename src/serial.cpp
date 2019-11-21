#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <inttypes.h>
#include <string.h>
#include "ros/ros.h"

int serial_fd = 0;

void serial_init(char *port_name, int baudrate)
{
	//open the port
	serial_fd = open(port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(serial_fd == -1) {
		ROS_FATAL("Failed to open the serial port.");
		exit(0);
	}

	//config the port
	struct termios options;

	tcgetattr(serial_fd, &options);

	options.c_cflag = CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;

	switch(baudrate) {
	case 9600:
		options.c_cflag |= B9600;
		break;
	case 57600:
		options.c_cflag |= B57600;
		break;
	case 115200:
		options.c_cflag |= B115200;
		break;
	default:
		ROS_INFO("Unknown baudrate. try 9600, 57600, 115200 or check \"serial.cpp\".");
		exit(0);
	}

	tcflush(serial_fd, TCIFLUSH);
	tcsetattr(serial_fd, TCSANOW, &options);
}

void serial_puts(char *s, size_t size)
{
	write(serial_fd, s, size);
}

void send_pose_to_serial(float pos_x_cm, float pos_y_cm, float pos_z_cm,
			 float quat_x, float quat_y, float quat_z, float quat_w)
{
	char *s = "c";
	serial_puts(s, strlen(s));
}
