/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/3 下午3:35
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/3 下午3:35
 * @Version 1.0
 */
#ifndef SRC_YOLO_DETECTION_H
#define SRC_YOLO_DETECTION_H

#include <algorithm>
#include <iostream>
#include <string>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

namespace YoloDetectorNS
{
class YoloDetector
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber bbox_sub_;
    ros::Subscriber rgb_image_sub_;

    ros::Publisher class_pub_;

    darknet_ros_msgs::BoundingBoxes yolo_data;
    double width;
    double height;

    std_msgs::String class_;

    // callback
    void bbox_cb(const darknet_ros_msgs::BoundingBoxesConstPtr& msg);

    // function
    void initROS();

public:
    YoloDetector();
    ~YoloDetector();
    void run();

};
}




#endif //SRC_YOLO_DETECTION_H
