/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : serial
 * @created     : Wednesday Nov 27, 2019 21:17:31 MST
 */

#ifndef TEENSY_MSG_H

#define TEENSY_MSG_H

#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>

#define TEENSY_BUFFER_SIZE ((sizeof(float) * 6) + sizeof(uint64_t) + sizeof(double))
#define CMD_VEL_BUFFER_SIZE (sizeof(float) * 2)

#ifdef __cplusplus
extern "C"
{
#endif

// An imu message
// contains covariance and
// reading (from a rolling average sample)
typedef struct {
  uint64_t id; // To help with dt
  double dt; // cange in time

  float angular_velocity[3];
  float linear_acceleration[3];
} imu_message;

// Input vector (linear and angular veloc)
typedef struct {
  float linear_v;
  float angular_v;
} cmd_vel;

// Serializable teensy message
typedef struct {
  cmd_vel vel;
  uint8_t buffer[CMD_VEL_BUFFER_SIZE];
} teensy_command;

void update_teensy_command(teensy_command* cmd, float lin, float ang);
void parse_imu_message(imu_message* msg, const uint8_t buffer[TEENSY_BUFFER_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* end of include guard SERIAL_H */
