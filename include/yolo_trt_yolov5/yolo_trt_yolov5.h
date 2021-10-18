/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/10/17 下午8:32
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/10/17 下午8:32
 * @Version 1.0
 */
#ifndef SRC_YOLO_TRT_YOLOV5_H
#define SRC_YOLO_TRT_YOLOV5_H

// c++
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <chrono>

// ros
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/image_encodings.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

// opencv
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include "NvInfer.h"
#include "utils.h"
#include "cuda_utils.h"
#include "logging.h"
#include "yololayer.h"

#define DEVICE 0
#define NMS_THRESH 0.4
#define BBOX_CONF_THRESH 0.5
#define BATCH_SIZE 1

using namespace std;
using namespace cv;
using namespace nvinfer1;

namespace YoloV5TensorRTNS
{
    class YoloV5TensorRT
    {
    private:
        int inputIndex;
        int outputIndex;
        void* buffers[2];
        cudaStream_t stream;

        IRuntime* runtime = nullptr;
        ICudaEngine* engine = nullptr;
        IExecutionContext* context = nullptr;
        char *trtModelStream{nullptr};
        size_t size{0};

        vector<string> classNamesVec;

        darknet_ros_msgs::BoundingBox box;
        darknet_ros_msgs::BoundingBoxes boundingBoxes;
        std_msgs::Header imageHeader_;

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Subscriber rgb_sub_;
        ros::Publisher bbox_pub_;

        // ros初始化
        void initROS();

        // function
        void loadClassNames();

        // 初始化推理引擎
        void loadEngine();
        // 开始推理
        void doInference(IExecutionContext& context, float* input, float* output, int batchSize);

        // callback
        void rgb_cb(const sensor_msgs::ImageConstPtr& msg);

    public:
        YoloV5TensorRT();
        ~YoloV5TensorRT();
        void run();
    };
}
#endif //SRC_YOLO_TRT_YOLOV5_H
