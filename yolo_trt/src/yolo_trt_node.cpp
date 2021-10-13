/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/10/8 下午7:57
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/10/8 下午7:57
 * @Version 1.0
 */
#include <yolo_trt/yolo_trt.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "yolo_trt_node");
    YoloTensorRTNS::YoloTensorRT app;
    app.run();
    return 0;
}