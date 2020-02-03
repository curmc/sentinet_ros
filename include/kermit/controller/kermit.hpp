#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

#include "kermit.hpp/common.h"

namespace rmt {
// DRIVE TRAIN, ROS METHODS, ROBOT CLASS

class Peripheral {
      public:
        virtual void stop() = 0;
        virtual void start() = 0;
        virtual void loop() = 0;
};

class DriveTrain : public Peripheral {
      public:
        void stop() override;
        void start() override;
        void loop() override;

        float getLinear() const;
        float getAngular() const;

      private:
        float linear_vel{0.0f};
        float angular_vel{0.0f};
};

class Robot {
      public:
        constexpr Robot();

        // Robot handles publishing linear and angular
        void publishLinAng();

      private:
        DriveTrain dt{};
        geometry_msgs::Twist twist{};
        ros::NodeHandle n{};
        ros::Publisher p{};
}

} // namespace rmt
