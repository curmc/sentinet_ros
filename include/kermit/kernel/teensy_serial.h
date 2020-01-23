/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : serial
 * @created     : Wednesday Nov 27, 2019 21:17:31 MST
 */

#ifndef TEENSY_SERIAL_H 

#define TEENSY_SERIAL_H 

#ifdef __cplusplus
extern "C"
{
#endif

#include "teensy_msg.h"
#include "serial.h"

typedef struct {
  imu_message msg;
  uint8_t buffer[TEENSY_BUFFER_SIZE];
} teensy_reading;

typedef struct
{
  int fd;
  teensy_command msg;
  teensy_reading resp_msg;
} teensy_device;

int new_teensy_device(teensy_device* device, const char* serialport);

int teensy_cleanup(teensy_device* device);

int teensy_write(teensy_device* device, float lin_accel, float ang_accel);

imu_message teensy_read(teensy_device* device);

#ifdef __cplusplus
}
#endif

#endif /* end of include guard TEENSY_SERIAL_H */
