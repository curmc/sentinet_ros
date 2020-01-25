/**
 * @author      : theo (theo.j.lincke@gmail.com)
 * @file        : AprilDetector
 * @created     : Wednesday Jan 01, 2020 11:33:21 MST
 */

#include "kermit/camera/AprilDetector.hpp"

#define PI 3.14159265358979323846264

#define DEBUGAPRIL

#ifdef DEBUGAPRIL
static bool verbose = true;
#else
static bool verbose = false;
#endif

AprilDetector::AprilDetector(const std::string type) {
        pub =
            n.advertise<geometry_msgs::PoseStamped>(topics::camera_topic, 1000);
        memset(&tag, 0, sizeof(tag));
        initialize_tag(type);
}

bool AprilDetector::sync_start() {
        video.cap = VideoCapture("/dev/video2");
        if (!video.cap.isOpened()) {
                ROS_ERROR("Couldnt open video device");
                return false;
        }
        return true;
}

bool AprilDetector::loop() {
        video.cap >> video.frame;
        cvtColor(video.frame, video.gray, COLOR_BGR2GRAY);

        image_u8_t image = {.width = video.gray.cols,
                            .height = video.gray.rows,
                            .stride = video.gray.cols,
                            .buf = video.gray.data};

        zarray_t *detections = apriltag_detector_detect(tag.td, &image);

        for (int i = 0; i < zarray_size(detections); ++i) {
                apriltag_pose_t pose;
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                tag.info.det = det;
                // double err = estimate_tag_pose(&tag.info, &pose);
                estimate_tag_pose(&tag.info, &pose);

                double *d = pose.R->data;

                // Convert R to a matrix3x3 in order to generate a quaternion
                tf2::Matrix3x3 a;
                a.setValue(d[0], d[1], d[2], d[3], d[4], d[5], d[6], d[7],
                           d[8]);

                double roll, pitch, yaw;
                a.getRPY(roll, pitch, yaw);

                // Create a quaternion
                tf2::Quaternion q;
                q.setRPY(roll, pitch, yaw);
                q.normalize();

                // Extract the quaternion message
                message.pose.orientation = tf2::toMsg(q);
                message.pose.position.x = pose.t->data[0];
                message.pose.position.y = pose.t->data[1];
                message.pose.position.z = pose.t->data[2];

                geometry_msgs::PoseStamped resp;
                message.header.stamp = ros::Time::now();
                message.header.frame_id = i; // TODO
                pub.publish(message);

                if (verbose) {
                        printf("Orientation: (%f %f %f)\n", roll, pitch, yaw);
                }
        }

        if (verbose) {
                for (int i = 0; i < zarray_size(detections); ++i) {
                        apriltag_detection_t *det;
                        zarray_get(detections, i, &det);
                        line(video.frame, Point(det->p[0][0], det->p[0][1]),
                             Point(det->p[1][0], det->p[1][1]),
                             Scalar(0, 0xff, 0), 2);
                        line(video.frame, Point(det->p[0][0], det->p[0][1]),
                             Point(det->p[3][0], det->p[3][1]),
                             Scalar(0, 0, 0xff), 2);
                        line(video.frame, Point(det->p[1][0], det->p[1][1]),
                             Point(det->p[2][0], det->p[2][1]),
                             Scalar(0xff, 0, 0), 2);
                        line(video.frame, Point(det->p[2][0], det->p[2][1]),
                             Point(det->p[3][0], det->p[3][1]),
                             Scalar(0xff, 0, 0), 2);

                        std::stringstream ss;
                        ss << det->id;
                        String text = ss.str();
                        int fontface = FONT_HERSHEY_SCRIPT_SIMPLEX;
                        double fontscale = 1.0;
                        int baseline;
                        Size textsize = getTextSize(text, fontface, fontscale,
                                                    2, &baseline);
                        putText(video.frame, text,
                                Point(det->c[0] - textsize.width / 2,
                                      det->c[1] + textsize.height / 2),
                                fontface, fontscale, Scalar(0xff, 0x99, 0), 2);
                }
        }

        apriltag_detections_destroy(detections);

        if (verbose) {
                imshow("Tag Detections", video.frame);
                if (waitKey(30) >= 0)
                        return false;
        }

        return true;
}

AprilDetector::~AprilDetector() {
        destroy_tag();
        destroy_video();
}

void AprilDetector::destroy_video() { return; }

void AprilDetector::destroy_tag() {
        if (tag.detections)
                apriltag_detections_destroy(tag.detections);
        if (tag.td)
                apriltag_detector_destroy(tag.td);

        tag.detections = NULL;
        tag.td = NULL;

        if (!tag.tf) {
                tag.type = 0;
                return;
        }

        switch (tag.type) {
        case (0):
                break;
        case (1):
                tag36h11_destroy(tag.tf);
                break;
        case (2):
                tag25h9_destroy(tag.tf);
                break;
        case (3):
                tag16h5_destroy(tag.tf);
                break;
        case (4):
                tagCircle21h7_destroy(tag.tf);
                break;
        case (5):
                tagCircle49h12_destroy(tag.tf);
                break;
        case (6):
                tagStandard41h12_destroy(tag.tf);
                break;
        case (7):
                tagStandard52h13_destroy(tag.tf);
                break;
        case (8):
                tagCustom48h12_destroy(tag.tf);
                break;
        }

        tag.tf = NULL;
        tag.type = 0;
}

bool AprilDetector::initialize_tag(const std::string type) {
        if (type == "tag36h11") {
                tag.tf = tag36h11_create();
                tag.type = 1;
        } else if (type == "tag25h9") {
                tag.tf = tag25h9_create();
                tag.type = 2;
        } else if (type == "tag16h5") {
                tag.tf = tag16h5_create();
                tag.type = 3;
        } else if (type == "tagCircle21h7") {
                tag.tf = tagCircle21h7_create();
                tag.type = 4;
        } else if (type == "tagCircle49h12") {
                tag.tf = tagCircle49h12_create();
                tag.type = 5;
        } else if (type == "tagStandard41h12") {
                tag.tf = tagStandard41h12_create();
                tag.type = 6;
        } else if (type == "tagStandard52h13") {
                tag.tf = tagStandard52h13_create();
                tag.type = 7;
        } else if (type == "tagCustom48h12") {
                tag.tf = tagCustom48h12_create();
                tag.type = 8;
        } else {
                ROS_ERROR("Error invalid tag type: %s", type.c_str());
                return false;
        }

        tag.td = apriltag_detector_create();
        apriltag_detector_add_family(tag.td, tag.tf);

        tag.td->quad_decimate = 2.0;
        tag.td->quad_sigma = 0.0;
        tag.td->debug = 0;
        tag.td->refine_edges = 1;
        tag.td->nthreads = 1;

        // Configured by the machine
        tag.info = (apriltag_detection_info_t){.det = NULL,
                                               .tagsize = 0.32,
                                               .fx = 1001.5520728,
                                               .fy = 1001.5520728,
                                               .cx = 640,
                                               .cy = 360};

        return true;
}
