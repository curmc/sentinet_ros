/**
 * @author                        : theo (theo.j.lincke@gmail.com)
 * @file                                : teensy_msg
 * @created                 : Tuesday Jan 07, 2020 11:51:19 MST
 */

#include "kermit/kernel/teensy_msg.h"

static uint16_t generate_checksum() { return rand() % 100000; }

/*
 Creates a new teensy packet for data
 Really just wipes data in the packet, so can also be used
 for restting
 */
int new_teensy_packet(teensy_packet *packet, int auto_checksum) {
        if (packet->state == BUSY) {
                return -1;
        } else if (packet->state == IDLE) {
                packet->state = BUSY;
                memset(packet, 0, sizeof(teensy_packet));
                packet->state = BUSY;

                if (auto_checksum) {
                        packet->cmd.auto_checksum = 1;
                }

                packet->state = IDLE;
                return 1;
        }

        return 1;
}

/*
 Serializes cmd data into the teensy_packet cmd_vel,
 doesn't set any data unless the checksum flag is set to true
 */
int teensy_serialize(teensy_packet *cmd) {

        if (cmd->state == BUSY) {
                return -1;
        }

        else if (cmd->state == IDLE) {
                cmd->state = BUSY;

                if (cmd->cmd.auto_checksum)
                        cmd->cmd.checksum = generate_checksum();

                uint16_t *checksum = (uint16_t *)cmd->cmd.buffer;
                *checksum++ = cmd->cmd.checksum;

                uint8_t *state = (uint8_t *)checksum;

                *state++ = (cmd->cmd.driving && 1);
                *state++ = (cmd->cmd.mining && 1);
                *state++ = (cmd->cmd.dumping && 1);

                float *vals = (float *)state;

                *vals++ = cmd->cmd.linear_v;
                *vals++ = cmd->cmd.angular_v;

                cmd->state = IDLE;
                return 1;
        }

        return -1;
}

/*
 Interprets incomming buffer as a teensy response and gets data from the buffer
 */
int teensy_deserialize(teensy_packet *resp,
                       const uint8_t buffer[TEENSY_BUFFER_SIZE]) {
        if (resp->state == BUSY) {
                return -1;
        }

        else if (resp->state == IDLE) {
                resp->state = BUSY;

                uint16_t *stats_handler = (uint16_t *)buffer;
                resp->resp.checksum = *stats_handler++;
                resp->resp.status = *stats_handler++;

                resp->state = IDLE;
                return 1;
        }

        return -1;
}
