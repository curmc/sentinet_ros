#include "kermit/kernel/teensy_serial.h"

static int validate_checksum(uint16_t output, uint16_t input) {
        return output == input;
}

/*
 Creates a new teensy device equiped with handler and data
 */
int new_teensy_device(teensy_device *device, const char *serialport) {
        if (device->state == BUSY) {
                return -1;
        } else if (device->state == IDLE) {

                device->state = BUSY;

                memset(device, 0, sizeof(teensy_device));
                device->fd = serialport_init(serialport, 9600);

                device->state = IDLE;

                return device->fd;
        }
        return -1;
}

// Sets the drive values in a teensy device - doesn't write to port
int teensy_set_drive(teensy_device *dev, float lin, float ang) {
        if (dev->state == BUSY) {
                return -1;
        } else if (dev->state == IDLE) {
                dev->state = BUSY;

                if (dev->data.cmd.mining || dev->data.cmd.dumping) {
                        printf("WARNING: Teensy state is mining / dumping\n");
                        return -1;
                }

                dev->data.cmd.dumping = 0;
                dev->data.cmd.mining = 0;

                dev->data.cmd.angular_v = ang;
                dev->data.cmd.linear_v = lin;

                dev->state = IDLE;

                return 1;
        }
        return -1;
}

/*
 Write to serial port macro for teensy drive train
 */
int teensy_write_drive(teensy_device *dev, float lin, float ang) {
        if (dev->state == BUSY) {
                return -1;
        } else if (dev->state == IDLE) {
                dev->state = BUSY;

                if (dev->data.cmd.mining || dev->data.cmd.dumping) {
                        printf("WARNING: Teensy state is mining / dumping\n");
                        return -1;
                }

                dev->data.cmd.dumping = 0;
                dev->data.cmd.mining = 0;

                dev->data.cmd.angular_v = ang;
                dev->data.cmd.linear_v = lin;

                teensy_serialize(&dev->data);

                int ret = serialport_write(dev->fd, dev->data.cmd.buffer,
                                           CMD_VEL_BUFFER_SIZE);

                ret = serialport_read(dev->fd, dev->data.resp.buffer,
                                      TEENSY_BUFFER_SIZE, 100);

                teensy_deserialize(&dev->data, dev->data.resp.buffer);

                int valid = validate_checksum(dev->data.resp.checksum,
                                              dev->data.cmd.checksum);

                dev->state = IDLE;
                if (!valid) {
                        printf("Teensy Error: Invalid Checksum\n");
                        return -1;
                }

                dev->state = IDLE;

                return ret;
        }
        return -1;
}

/*
 Write to serial port macro for teensy state
 */
int teensy_set_state(teensy_device *dev, robot_state state) {
        if (dev->state == BUSY) {
                return -1;
        } else if (dev->state == IDLE) {
                dev->state = BUSY;
                if (state == DRIVING) {
                        dev->data.cmd.driving = 1;
                        dev->data.cmd.dumping = 0;
                        dev->data.cmd.mining = 0;
                }

                else if (state == MINING || state == DUMPING) {
                        dev->data.cmd.driving = 0;
                        dev->data.cmd.linear_v = 0.0;
                        dev->data.cmd.angular_v = 0.0;
                        dev->data.cmd.mining = (state == MINING ? 1 : 0);
                        dev->data.cmd.dumping = (state == DUMPING ? 1 : 0);
                }

                dev->state = IDLE;

                return 1;
        }
        return -1;
}

/*
 Write to serial port macro for teensy state
 */
int teensy_write_state(teensy_device *dev, robot_state state) {
        if (dev->state == BUSY) {
                return -1;
        } else if (dev->state == IDLE) {
                dev->state = BUSY;
                if (state == DRIVING) {
                        dev->data.cmd.driving = 1;
                        dev->data.cmd.dumping = 0;
                        dev->data.cmd.mining = 0;
                }

                else if (state == MINING || state == DUMPING) {
                        dev->data.cmd.driving = 0;
                        dev->data.cmd.linear_v = 0.0;
                        dev->data.cmd.angular_v = 0.0;
                        dev->data.cmd.mining = (state == MINING ? 1 : 0);
                        dev->data.cmd.dumping = (state == DUMPING ? 1 : 0);
                }

                teensy_serialize(&dev->data);

                int ret = serialport_write(dev->fd, dev->data.cmd.buffer,
                                           CMD_VEL_BUFFER_SIZE);

                ret = serialport_read(dev->fd, dev->data.resp.buffer,
                                      TEENSY_BUFFER_SIZE, 100);

                teensy_deserialize(&dev->data, dev->data.resp.buffer);

                int valid = validate_checksum(dev->data.resp.checksum,
                                              dev->data.cmd.checksum);

                dev->state = IDLE;
                if (!valid) {
                        printf("Teensy Error: Invalid Checksum\n");
                        return -1;
                }

                return ret;
        }
        return -1;
}

int teensy_send(teensy_device *dev) {

        if (dev->state == BUSY) {
                return -1;
        } else if (dev->state == IDLE) {
                dev->state = BUSY;
                teensy_serialize(&dev->data);
                int ret = serialport_write(dev->fd, dev->data.cmd.buffer,
                                           CMD_VEL_BUFFER_SIZE);
                dev->state = IDLE;
                return ret;
        }
        return -1;
}

int teensy_recieve(teensy_device *dev, uint16_t checksum) {

        if (dev->state == BUSY) {
                return -1;
        } else if (dev->state == IDLE) {
                dev->state = BUSY;
                int ret = serialport_read(dev->fd, dev->data.resp.buffer,
                                          TEENSY_BUFFER_SIZE, 100);

                teensy_deserialize(&dev->data, dev->data.resp.buffer);

                int valid =
                    validate_checksum(dev->data.resp.checksum, checksum);

                dev->state = IDLE;
                if (!valid) {
                        printf("Teensy Error: Invalid Checksum\n");
                        return -1;
                }

                dev->state = IDLE;

                return ret;
        }
        return -1;
}

/*
 Clean teensy state up
 */
int teensy_cleanup(teensy_device *device) {
        return serialport_close(device->fd);
}
