/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/14 下午8:45
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/14 下午8:45
 * @Version 1.0
 */
#ifndef SRC_PEOPLE_TRACKING_H
#define SRC_PEOPLE_TRACKING_H

// C++
#include <algorithm>
#include <chrono>
#include <iostream>

// ROS
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

// Opencv
#include <opencv2/opencv.hpp>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

#include "kcftracker.hpp"

using namespace std;
using namespace cv;

namespace PeopleTrackNS
{
class PID
{
public:
    double e0;
    double e1;

    double p;
    double d;
    PID()
    {
        e0 = 0;
        e1 = 0;
        p = 0;
        d = 0;
    }
};

class PeopleTrack
{
     private:
            ros::NodeHandle pnh_;
            ros::NodeHandle nh_;

            // Subscriber
            ros::Subscriber bbox_sub_;
            ros::Subscriber rgb_image_sub_;
            ros::Subscriber depth_image_sub_;
            ros::Subscriber end_sub_;

            // Publisher
            ros::Publisher twist_pub_;
            ros::Publisher tts_pub_;
            ros::Publisher state_pub_;

            // image
            Mat rgbimage;
            Mat depthimage;

            // KCF
            bool kcf_init_;
            bool enable_get_depth;
            KCFTracker kcfTracker_;
            Rect rect_;

            // Tracking Params
            double target_distance_;
            bool twist_stop_;
            bool is_tracking_;
            bool always_follow_;
            string target_object_;

            // image error
            int ERROR_OFFSET_X_LEFT1 = 200;
            int ERROR_OFFSET_X_LEFT2 = 615;
            int ERROR_OFFSET_X_RIGHT1 = 665;
            int ERROR_OFFSET_X_RIGHT2 = 1080;

            // PID
            PID pid_speed;
            PID pid_angle;

            // for pid bias
            double target_max_distance_;
            double distance_min_bias_;
            double distance_max_bias_;

            // speed limit for PID
            double max_linear_speed_;
            double min_linear_spped_;
            double base_speed_;

            // angle limit for PID
            double max_angle_speed_;
            double k_rotation_speed_;

            // callback
            void rgb_cb(const sensor_msgs::ImageConstPtr& msg);
            void depth_cb(const sensor_msgs::ImageConstPtr& msg);
            void bbox_cb(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);
            void end_cb(const std_msgs::BoolConstPtr& msg);

            // function
            double PID_control(double now, double target, PID pid);
            void speak(String str);

            // init
            void initROS();

    public:
            PeopleTrack();
            ~PeopleTrack();
            void run();
};

}

#endif //SRC_PEOPLE_TRACKING_H
