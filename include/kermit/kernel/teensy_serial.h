/**
 * @author                        : theo (theo.j.lincke@gmail.com)
 * @file                                : serial
 * @created                 : Wednesday Nov 27, 2019 21:17:31 MST
 */

#ifndef TEENSY_SERIAL_H 

#define TEENSY_SERIAL_H 

#ifdef __cplusplus
extern "C"
{
#endif

#include "teensy_msg.h"
#include "serial.h"

typedef enum {MINING, DRIVING, DUMPING} robot_state;

typedef struct
{
        int fd;
        teensy_packet data;
        teensy_data_state state;
} teensy_device;

int new_teensy_device(teensy_device* device, const char* serialport);
int teensy_cleanup(teensy_device* device);




/*
 * write is the same thing as acting:
 * set, send, recieve
 *
 * I've just added smaller methods so you have access to implimenations
 *
 * In write, checksum is automatically validated with the sent checksum
 * wherease in recieve you provide it yourself
 */
int teensy_write_drive(teensy_device* device, float lin_accel, float ang_accel);
int teensy_write_state(teensy_device* device, robot_state state);

int teensy_set_drive(teensy_device* device, float lin_accel, float ang_accel);
int teensy_set_state(teensy_device* device, robot_state state);

int teensy_send(teensy_device* device);
int teensy_recieve(teensy_device* device, uint16_t checksum);

#ifdef __cplusplus
}
#endif

#endif /* end of include guard TEENSY_SERIAL_H */
