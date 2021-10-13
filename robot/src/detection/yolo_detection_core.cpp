/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/3 下午3:36
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/3 下午3:36
 * @Version 1.0
 */

#include <detection/yolo_detection.h>

namespace YoloDetectorNS
{
    YoloDetector::YoloDetector()
        : nh_()
        , pnh_("~")
        , height(720)
        , width(1280)
    {
        initROS();
    }

    YoloDetector::~YoloDetector()
    {

    }

    void YoloDetector::initROS()
    {
        bbox_sub_ = nh_.subscribe("yolo_trt/bounding_boxes", 1, &YoloDetector::bbox_cb, this);

        class_pub_ = nh_.advertise<std_msgs::String>("object_class", 1);
    }

    void YoloDetector::bbox_cb(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& ex)
        {
            ROS_ERROR("cv_bridge exception %s", ex.what());
            return;
        }

        for (auto& box : msg->bounding_boxes)
        {
            rectangle(cv_ptr->image, Point(box.xmin, box.ymin), Point(box.xmax, box.ymax), Scalar(255, 255, 0), 2);
            putText(cv_ptr->image, box.Class, Point(box.xmin, box.ymin), FONT_HERSHEY_PLAIN, 1.0, Scalar(255, 255, 255), 1);
        }
        imshow("test", cv_ptr->image);
        waitKey(1);

        for (auto& box : msg->bounding_boxes)
        {
            class_.data = box.Class;
            class_pub_.publish(class_);
        }

    }

    void YoloDetector::run()
    {
        while (ros::ok())
        {
            ros::spinOnce();
        }
    }
}