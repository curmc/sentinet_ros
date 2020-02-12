#include "kermit/controller/kermit.hpp"

namespace rmt {

void DriveTrain::start() {
        linear_vel = 0.0f;
        angular_vel = 0.0f;
}

void DriveTrain::stop() {
        linear_vel = 0.0f;
        angular_vel = 0.0f;
}

void DriveTrain::moveForward(){

}

void DriveTrain::loop() {}

} // namespace rmt


//write publisher that publishes these basic values to cmd_vel_topic
//publish the geometry_messages_twist twist
//Look at wii controller.cpp under nodes for a reference on how to publish things
//Go into nodes and make an executable, add to Cmake, call functions
