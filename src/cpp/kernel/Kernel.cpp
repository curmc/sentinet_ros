/**
 * @author                        : theo (theo.j.lincke@gmail.com)
 * @file                                : Kernel
 * @created                 : Tuesday Jan 07, 2020 12:57:01 MST
 */

#include "kermit/kernel/Kernel.hpp"

Kernel::Kernel() {
        // New subscriber on cmd vel
        sub = n.subscribe(topics::cmd_vel_topic, 1000,
                          &Kernel::cmd_vel_callback, this);
}

Kernel::~Kernel() { 
        serial_device_cleanup(&dev);
        teensy_cleanup();
}

bool Kernel::initialize_teensy(const std::string &port) {

        teensy_init();
        int ret;

        if ((ret = new_serial_device(&dev, port.c_str())) == -1)
                return ret;

        ROS_INFO("Teensy device started on port %s", port.c_str());

        return true;
}

void Kernel::cmd_vel_callback(const geometry_msgs::Twist &msg) {

        float lin = msg.linear.x; 
        float ang = msg.angular.z;

        bool status = false;

        status = teensy_callback(lin, ang);

        if (!status){
                std::cout<<"Error. Showing most recent Command State:"<<std::endl;
        }

        return;
}

bool Kernel::teensy_callback(float lin, float ang) {

        to_msg.ang = ang;
        to_msg.lin = lin;

        int ret = t_send(&to_msg, &dev);
        
        if(ret == FAILURE)
                ROS_WARN("Warning, message was not written to serial device");
        
        return ret == SUCCESS;
}


static double angle_normalize(double theta) {
        while (theta > 3.14) {
                theta -= 2.0 * 3.14;
        }
        while (theta < -3.14) {
                theta += 2.0 * 3.14;
        }
        return theta;
}
