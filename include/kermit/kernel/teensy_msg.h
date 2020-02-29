#ifndef TEENSY_MSG_H

#define TEENSY_MSG_H

#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>

#define TEENSY_BUFFER_SIZE 4
#define CMD_VEL_BUFFER_SIZE 13

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Teensy command is the command sent onto the wire
 * to the teensy - contains a state vector and 
 * linear / angular velocity commands
 */
typedef struct {
        // Not on the wire
        uint8_t auto_checksum;

        // 2 bytes
        uint16_t checksum;
        // 3 bytes
        uint8_t driving;
        uint8_t mining;
        uint8_t dumping;
        // 4 bytes
        float linear_v;
        // 4 bytes
        float angular_v;
        uint8_t buffer[CMD_VEL_BUFFER_SIZE];
} teensy_command;


/*
 * The teensy response is the response from the wire 
 * for now it only contains a status and checksum 
 */
typedef struct {
        // 2 bytes
        uint16_t checksum;
        // 2 bytes
        uint16_t status;

        // Here is where the rest of the teensy sensor data goes

        uint8_t buffer[TEENSY_BUFFER_SIZE];
} teensy_response;



/*
 * When reading from the wire, the writing can either be busy or idle
 */
typedef enum { BUSY = 1, IDLE = 0 } teensy_data_state;


/*
 * The main endpoint to handle everything teensy data wise
 */
typedef struct {
        teensy_response resp;
        teensy_command cmd;
        teensy_data_state state;
} teensy_packet;

/*
 * Create a new collection of teensy stuff
 * if auto checksum is true, generates a checksum every time
 */
int new_teensy_packet(teensy_packet *packet, int auto_checksum);

/*
 * Serializes the data into the teensy message (to wire)
 */
int teensy_serialize(teensy_packet *cmd);

/*
 * Deserializes from the teensy wire
 */
int teensy_deserialize(teensy_packet *resp,
                       const uint8_t buffer[TEENSY_BUFFER_SIZE]);

#ifdef __cplusplus
}
#endif

#endif /* end of include guard SERIAL_H */
