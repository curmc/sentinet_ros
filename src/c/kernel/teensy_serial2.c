/**
 * @author                        : theo (theo.j.lincke@gmail.com)
 * @file                                : teensy_msg
 * @created                 : Tuesday Jan 07, 2020 11:51:19 MST
 */

#include "kermit/kernel/teensy_serial.h"
#include <stdlib.h>

static uint16_t generate_checksum() { return rand() % 100000; }

static teensy_device device;
static int to_teensy_serialize(to_teensy_msg* msg, teensy_device* dev);
static int from_teensy_deserialize(from_teensy_msg* msg, teensy_device* dev);

serial_status t_send(to_teensy_msg* msg, serial_dev_attr* dev) {

        // Serialize the message
        if(!to_teensy_serialize(msg, &device)) {
                return FAILURE;
        }
        
        // Write data
        pthread_mutex_lock(&dev->serial_lock);
        int ret = serialport_write(dev->fd, device.w_buffer, TO_TEENSY_MSG_SIZE);
        pthread_mutex_unlock(&dev->serial_lock);

        // serialport write returns -1 on failure else 0
        if(ret < 0)
                return SUCCESS;

        return FAILURE;
}

serial_status t_recieve(from_teensy_msg* msg, serial_dev_attr* dev) {

        // Read from the device
        pthread_mutex_lock(&dev->serial_lock);
        int ret = serialport_read(dev->fd, device.r_buffer, TO_TEENSY_MSG_SIZE, 100);
        pthread_mutex_unlock(&dev->serial_lock);


        // Serialport read returns 0 on success, else -1 or -2
        if(ret == -2)
                return TIMEOUT;

        if(ret == -1)
                return NOREAD;

        if(ret == 0){
                if(!from_teensy_deserialize(msg, &device)) {
                        return FAILURE;
                }
                return SUCCESS;
        }

        return FAILURE;
}
