/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : common
 * @created     : Tuesday Jan 07, 2020 13:52:33 MST
 */

#ifndef COMMON_H

#define COMMON_H

#include <string>

// TODO Change this to  ayaml file
namespace topics {

/*
 * Processed data topic
 */
const std::string cmd_vel_topic = "kermit/cmd_vel";
const std::string localizer_topic = "kermit/state_localizer";
const std::string path_state_topic = "kermit/path_state";


/*
 * Sensor topics
 */
const std::string imu_topic = "imu_message";
const std::string camera_topic = "camera";
}

namespace tf_frame_ids {
const uint32_t teensy_1 = 1;
}

namespace nodes {
const std::string Localizer_Node = "Localizer";
const std::string Camera_Node = "Camera";
const std::string Kernel_Node = "Kernel";
const std::string Controller_Node = "Controller";
}


#endif /* end of include guard COMMON_H */

