/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/10/17 下午8:47
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/10/17 下午8:47
 * @Version 1.0
 */

#include <yolo_trt_yolov5/yolo_trt_yolov5.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "yolov5_trt_node");
    YoloV5TensorRTNS::YoloV5TensorRT app;
    app.run();
    return 0;
}