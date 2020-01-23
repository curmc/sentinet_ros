/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : teensy_msg
 * @created     : Tuesday Jan 07, 2020 11:51:19 MST
 */

#include "kermit/kernel/teensy_msg.h"

void update_teensy_command(teensy_command* cmd, float lin, float ang) {
  float* handle = (float*)&cmd->buffer[0];
  *handle++ = lin;
  *handle = ang;
}

void parse_imu_message(imu_message* msg, const uint8_t buffer[TEENSY_BUFFER_SIZE]){
  /*
   * Deserialize the id of the message
   */
  uint64_t* id = (uint64_t*)&buffer[0];
  msg->id = *id++;

  /*
   * Deserialize the time stamp (a double)
   */
  double* dt = (double*)id;
  msg->dt = *dt++;

  /* 
   * Deserialize the actual data
   */
  float* handle = (float*)dt;

  // ACTUAL IMU READINGS
  for(size_t i = 0; i < 3; ++i)
    msg->linear_acceleration[i] = *handle++;
  for(size_t i = 0; i < 3; ++i)
    msg->angular_velocity[i] = *handle++;
}
