#include <stdio.h>
#include <string.h>
// Linux headers
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h>  // write(), read(), close()
// ROS
// #include <ros.h>
// #include <std_msgs/Float64MultiArray.h>

int main()
{
  int serial_port = open("/dev/ttyACM0", O_RDWR);
  struct termios tty;
  // Check for errors
  if (serial_port < 0)
  {
    printf("Error %i from open: %s\n", errno, strerror(errno));
  }
  if (tcgetattr(serial_port, &tty) != 0)
  {
    printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }
  // control modes
  tty.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity
  tty.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication
  tty.c_cflag |= CS8;            // 8 bits per byte
  tty.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)
  cfsetispeed(&tty, B9600);
  char read_buf[256];
  int n = read(serial_port, &read_buf, sizeof(read_buf));
  return 0;
}
