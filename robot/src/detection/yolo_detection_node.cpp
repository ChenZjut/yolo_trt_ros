/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/3 下午3:35
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/3 下午3:35
 * @Version 1.0
 */

#include <detection/yolo_detection.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detection_node");
    YoloDetectorNS::YoloDetector app;
    app.run();
    return 0;
}