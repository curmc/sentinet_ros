/**
 * @author                        : theo (theo.j.lincke@gmail.com)
 * @file                                : serial
 * @created                 : Wednesday Nov 27, 2019 21:17:31 MST
 */

#ifndef TEENSY_MSG_H

#define TEENSY_MSG_H

#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#define TEENSY_BUFFER_SIZE ((sizeof(uint16_t) * 2))
#define CMD_VEL_BUFFER_SIZE (sizeof(float) * 2 + 3 + sizeof(uint16_t))

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct {
        uint8_t auto_checksum;

        uint16_t checksum;

        uint8_t driving;
        uint8_t mining;
        uint8_t dumping;

        float linear_v;
        float angular_v;
        uint8_t buffer[CMD_VEL_BUFFER_SIZE];
} teensy_command;

typedef struct {
        uint16_t checksum;
        uint16_t status;

        // Here is where the rest of the teensy sensor data goes
        
        uint8_t buffer[TEENSY_BUFFER_SIZE];
} teensy_response;

typedef enum {BUSY = 1, IDLE = 0} teensy_data_state;
typedef struct {
        teensy_response resp;
        teensy_command cmd;
        teensy_data_state state;
        
} teensy_packet;

int new_teensy_packet(teensy_packet* packet, int auto_checksum);
int teensy_serialize(teensy_packet* cmd);
int teensy_deserialize(teensy_packet* resp, const uint8_t buffer[TEENSY_BUFFER_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* end of include guard SERIAL_H */
