/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : camera
 * @created     : Tuesday Jan 07, 2020 15:03:51 MST
 */

// If debug april is on
// it prints the state
// and shows the camera stream
#include "kermit/camera/AprilDetector.hpp"

/*
 * This is the
 * simulation file
 * so no serial here
 */
int main(int argc, char **argv) {
        ros::init(argc, argv, nodes::Camera_Node);

        AprilDetector detector("tag36h11");

        detector.sync_start();

        ros::Rate loop_rate(100);

        while (ros::ok()) {
                detector.loop();
                ros::spinOnce();
                loop_rate.sleep();
        }

        return 0;
}
