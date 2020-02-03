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

void DriveTrain::loop() {}

} // namespace rmt