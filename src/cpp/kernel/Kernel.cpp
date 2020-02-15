/**
 * @author                        : theo (theo.j.lincke@gmail.com)
 * @file                                : Kernel
 * @created                 : Tuesday Jan 07, 2020 12:57:01 MST
 */

#include "kermit/kernel/Kernel.hpp"

Kernel::Kernel(bool debug_, bool simulation_) {

        // New subscriber on cmd vel
        sub = n.subscribe(topics::cmd_vel_topic, 1000,
                          &Kernel::cmd_vel_callback, this);

        // pub = n.advertise<kermit::mars_imu>(topics::imu_topic, 1000);

        simulation = simulation_;

        if (simulation) {
                simul_state = std::unique_ptr<KermitSimulation>(
                    new KermitSimulation(n, pub));
        }

        debug = debug_;
}

Kernel::~Kernel() { teensy_cleanup(&dev); }

bool Kernel::initialize_teensy(const std::string &port) {

        if (simulation) {
                ROS_WARN("Started kernel in simulation mode, changing to "
                         "actual mode");
                simulation = false;
        }

        int ret;

        if ((ret = new_teensy_device(&dev, port.c_str())) == -1)
                return ret;

        debug = false;
        ROS_INFO("Teensy device started on port %s", port.c_str());

        return true;
}

bool Kernel::teensy_callback(float lin, float ang) {
        int ret = teensy_write_drive(&dev, lin, ang);

        if (ret == -1)
                ROS_WARN("Warning, message was not written to serial device");

        return ret;
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

static double rad_to_deg(double rad) { return 180 * (rad / 3.14); }

bool Kernel::KermitSimulation::cmd_vel_callback(float lin, float ang) {

        // Get delta time
        steady_clock::time_point prev = timer;
        timer = steady_clock::now();
        double dt = double(duration_cast<nanoseconds>(timer - prev).count()) /
                    1000000000;

        // Overflow TODO
        if (timer < prev)
                return false;

        // Two cases to avoid nans, radius = infinity (ang = 0) or radius = v /
        // w
        if (angular == 0.0) {

                // Theoretical x and y position (account for nan for rare cases)
                double x_pos_ = x_pos + cos(yaw) * veloc * dt;
                double y_pos_ = y_pos + sin(yaw) * veloc * dt;

                // TODO is this necessary?
                x_pos = (std::isnan(x_pos) ? x_pos : x_pos_);
                y_pos = (std::isnan(y_pos) ? y_pos : y_pos_);

        } else {

                assert(!std::isnan(veloc));
                assert(!std::isnan(angular));

                float r = veloc / angular;

                // TODO is this necessary?
                assert(!std::isnan(r));

                // Update yaw
                float yaw_new = yaw + angular * dt;

                // Update x and y position by angular rotation
                double x_pos_ = x_pos + r * (cos(yaw_new) - cos(yaw));
                double y_pos_ = y_pos + r * (sin(yaw_new) - sin(yaw));

                // TODO is this necessary?
                x_pos = (std::isnan(x_pos) ? x_pos : x_pos_);
                y_pos = (std::isnan(y_pos) ? y_pos : y_pos_);

                // Normailze yaw by constraining in [-pi, pi]
                yaw = angle_normalize(yaw_new);
        }

        imu.angular_velocity.x = 0.0;
        imu.angular_velocity.y = 0.0;
        imu.angular_velocity.z = ang;

        imu.linear_acceleration.x = 0.0;
        imu.linear_acceleration.y = 0.0;
        imu.linear_acceleration.z = 0.0;

        imu.pitch = 0.0;
        imu.roll = 0.0;
        // Publish theoretical imu data
        imu_pub->publish(imu);

        veloc = lin;
        angular = ang;

        pos.header.frame_id = 1;
        pos.header.stamp = ros::Time::now();
        pos.pose.position.x = x_pos;
        pos.pose.position.y = y_pos;
        pos.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pos.pose.orientation = tf2::toMsg(q);

        camera_simul.publish(pos);
        return true;
}

Kernel::KermitSimulation::KermitSimulation(ros::NodeHandle &handle,
                                           ros::Publisher &imu_pub_) {
        camera_simul = handle.advertise<geometry_msgs::PoseStamped>(
            topics::camera_topic, 10);
        imu_pub = &imu_pub_;

        timer = steady_clock::now();
        x_pos = 0.0;
        y_pos = 0.0;
        veloc = 0.0;
        angular = 0.0;
        yaw = 0.0;
}

Kernel::KermitSimulation::~KermitSimulation() {}

void Kernel::cmd_vel_callback(const geometry_msgs::Twist &msg) {
        if (debug) {
                debug_printf(msg);
        }

        float lin = msg.linear.x;
        float ang = msg.angular.z;

        bool status = false;

        if (simulation)
                status = simul_state->cmd_vel_callback(lin, ang);
        else
                status = teensy_callback(lin, ang);

        if (!status) {
                std::cout << "Error. Showing most recent Command State:"
                          << std::endl;
                debug_printf(msg);
        }

        return;
}

void Kernel::debug_printf(const geometry_msgs::Twist &) {
        return;
        // ROS_INFO("Linear: %f %f %f", msg.linear.x, msg.linear.y,
        // msg.linear.z); ROS_INFO("Angular: %f %f %f", msg.angular.x,
        // msg.angular.y, msg.angular.z);
}
