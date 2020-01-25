/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : LocalizerFilter
 * @created     : Tuesday Jan 07, 2020 15:57:38 MST
 */

#include "kermit/localizer_filter/LocalizerFilter.hpp"

LocalizerFilter::LocalizerFilter() {
        imu = nh.subscribe(topics::imu_topic, 1000,
                           &LocalizerFilter::imu_callback, this);
        camera = nh.subscribe(topics::camera_topic, 100,
                              &LocalizerFilter::camera_callback, this);
        path_state =
            nh.advertise<kermit::path_state>(topics::path_state_topic, 1000);
}

LocalizerFilter::~LocalizerFilter() {}

void LocalizerFilter::imu_callback(const kermit::mars_imu &msg) {
        state.path_state.ang.x = msg.roll;
        state.path_state.ang.y = msg.pitch;
        state.path_state.ang_vel = msg.angular_velocity.z;
}

void LocalizerFilter::camera_callback(const geometry_msgs::PoseStamped &msg) {

        tf2::Quaternion quat;
        tf2::fromMsg(msg.pose.orientation, quat);

        double pitch, roll, yaw;
        tf2::Matrix3x3 m(quat);
        m.getRPY(roll, pitch, yaw);

        state.path_state.lin.x = msg.pose.position.x;
        state.path_state.lin.y = msg.pose.position.y;
        state.path_state.lin.z = msg.pose.position.z;

        // Publish once we hear from april tags
        path_state.publish(state.path_state);
}
