#include "kermit/kernel/teensy_serial.h"

int
new_teensy_device(teensy_device* device, const char* serialport)
{
  memset(device, 0, sizeof(teensy_device));
  device->fd = serialport_init(serialport, 9600);
  return device->fd;
}

int
teensy_write(teensy_device* dev, float lin, float ang)
{
  update_teensy_command(&dev->msg, lin, ang);
  return serialport_write(dev->fd, dev->msg.buffer, CMD_VEL_BUFFER_SIZE);
}

imu_message teensy_read(teensy_device* dev) {
  imu_message ret;
  memset(&ret, 0, sizeof(imu_message));

  // Reading from the serial port
  int status;
  if((status = serialport_read(dev->fd, dev->resp_msg.buffer, TEENSY_BUFFER_SIZE, 1)) < 0) {
    printf("ERROR\n");
    return ret;
  }
  /** for(size_t i = 0; i < TEENSY_BUFFER_SIZE; ++i) { */
  /**   printf("%x ", dev->resp_msg.buffer[i]); */
  /** } */
  /** printf("\n"); */
  /**  */
  // Update imu_message values
  parse_imu_message(&dev->resp_msg.msg, dev->resp_msg.buffer);

  // Serial read / writes are at risk of race conditions
  // so create a new msg entirely
  memcpy(&ret, &dev->resp_msg.msg, sizeof(imu_message));
  return ret;
}

int
teensy_cleanup(teensy_device* device)
{
  return serialport_close(device->fd);
}
