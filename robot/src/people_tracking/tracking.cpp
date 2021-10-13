/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/17 上午9:57
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/17 上午9:57
 * @Version 1.0
 */
#include <people_tracking/people_tracking.h>

namespace PeopleTrackNS
{
    PeopleTrack::PeopleTrack()
        : nh_()
        , pnh_("~")
        , kcfTracker_(true, false, true, false)
        , kcf_init_(false)
        , target_object_("people")
        , enable_get_depth(false)
        , is_tracking_(false)
    {
        initROS();
    }

    PeopleTrack::~PeopleTrack()
    {

    }

    void PeopleTrack::initROS()
    {
        pnh_.param<double>("max_angle_speed", max_angle_speed_, 1.0);
        pnh_.param<double>("base_speed", base_speed_, 0.55);
        pnh_.param<double>("k_rotation_speed", k_rotation_speed_, 0.005);
        pnh_.param<double>("target_distance", target_distance_, 0.6);
        pnh_.param<double>("target_max_distance_", target_max_distance_, 1.0);
        pnh_.param<double>("speed_Kp", pid_speed.p, 0.8);
        pnh_.param<double>("speed_Kd", pid_speed.d, 0.2);
        pnh_.param<double>("angle_Kp", pid_angle.p, 0.8);
        pnh_.param<double>("angle_Kd", pid_angle.d, 0.2);

        rgb_image_sub_ = nh_.subscribe("cv_camera/image_raw", 10, &PeopleTrack::rgb_cb, this);
        depth_image_sub_ = nh_.subscribe("depth_to_rgb/image_raw", 10, &PeopleTrack::depth_cb, this);
        bbox_sub_ = nh_.subscribe("yolo_trt/bounding_boxes", 10, &PeopleTrack::bbox_cb, this);
        end_sub_ = nh_.subscribe("tracking_end", 10, &PeopleTrack::end_cb, this);

        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    }

    void PeopleTrack::end_cb(const std_msgs::BoolConstPtr &msg)
    {
        is_tracking_ = msg->data;
    }

    void PeopleTrack::bbox_cb(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
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

        if (kcf_init_ == false)
        {
            int yolo_bbox_n = 0;
            Rect rect;
            for (auto& box : msg->bounding_boxes)
            {
                if (box.Class == target_object_)
                {
                    rect.x = box.xmin;
                    rect.y = box.ymin;
                    rect.width = box.xmax - box.xmin;
                    rect.height = box.ymax - box.ymin;

                    yolo_bbox_n++;
                }
            }
            if (yolo_bbox_n == 1)
            {
                kcfTracker_.init(rect, cv_ptr->image);
                rect_ = rect;
                kcf_init_ = true;
            }
        }
         else
         {
             for (auto& box : msg->bounding_boxes)
             {
                 double width = box.xmax - box.xmin;
                 double height = box.ymax - box.xmin;
                 if ((rect_.width * rect_.height) / (width * height) < 0.8)
                 {
                     Rect rect(box.xmin, box.ymin, box.xmax - box.xmin, box.ymax - box.ymin);
                     kcfTracker_.init(rect, cv_ptr->image);
                     break;
                 }
             }
         }
    }

    void PeopleTrack::rgb_cb(const sensor_msgs::ImageConstPtr &msg)
    {
        if ((!kcf_init_) || is_tracking_)
            return;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv_ptr->image.copyTo(rgbimage);
        }
        catch (cv_bridge::Exception ex)
        {
            ROS_ERROR("rgb exception: %s", ex.what());
            return;
        }

        if (kcf_init_)
        {
            cout << "test2" << endl;
            enable_get_depth = true;
            rect_ = kcfTracker_.update(rgbimage);
            rectangle(rgbimage, rect_, Scalar(255, 255, 0), 1, 8);
        }

        imshow("RGB", rgbimage);
        waitKey(1);
    }

    void PeopleTrack::depth_cb(const sensor_msgs::ImageConstPtr &msg)
    {
        geometry_msgs::Twist speed_cmd;
        if ((!kcf_init_) || is_tracking_)
        {
            speed_cmd.linear.x = 0;
            speed_cmd.angular.z = 0;
            twist_pub_.publish(speed_cmd);
            return;
        }

        cv_bridge::CvImagePtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            cv_ptr->image.copyTo(depthimage);
        }
        catch (cv_bridge::Exception& ex)
        {
            ROS_ERROR("depth exception: %s", ex.what());
            return;
        }

        vector<double>dist_val(5);
        if (enable_get_depth)
        {
            dist_val[0] = depthimage.at<float>(rect_.y+rect_.height/3, rect_.x+rect_.width/3);
            dist_val[1] = depthimage.at<float>(rect_.y+rect_.height/3, rect_.x+2*rect_.width/3);
            dist_val[2] = depthimage.at<float>(rect_.y+2*rect_.height/3, rect_.x+rect_.width/3);
            dist_val[3] = depthimage.at<float>(rect_.y+2*rect_.height/3, rect_.x+2*rect_.x*rect_.height/3);
            dist_val[4] = depthimage.at<float>(rect_.y+rect_.height/2, rect_.x+rect_.width/2);

            float distance = 0;
            int num_depth_points = dist_val.size();
            for (size_t i = 0; i < dist_val.size(); i++)
            {
                if (dist_val[i] > 0.4 && dist_val[i] < 8.0)
                    distance += dist_val[i];
                else
                    num_depth_points--;
            }

            distance /= num_depth_points;

            if (isnan(distance))
                distance  = 0;

            if (distance == 0)
            {
                cout << "depth is zero !" <<endl;
                speed_cmd.linear.x = 0;
                speed_cmd.angular.z = 0;
                twist_pub_.publish(speed_cmd);
            }

            double speed = 0.0;
            double last_speed = 0.0;
            double now_speed = 0.0;
            double diff_speed;
            diff_speed = fabs(last_speed) - fabs(speed);
            if (distance < target_distance_)
            {
                distance_min_bias_ = (target_distance_ + target_max_distance_)/2 -distance;
                now_speed = -0.9 * (distance_min_bias_ * base_speed_ * pid_speed.p + diff_speed * pid_speed.d);
            }
            else if (distance > target_max_distance_ && distance < 4.0)
            {
                distance_max_bias_ = distance - (target_distance_ + target_max_distance_)/2;
                now_speed = distance_max_bias_ * base_speed_ * pid_speed.p + diff_speed * pid_speed.d;
            }
            else
                now_speed = 0;

            last_speed = speed;
            speed = now_speed;

            int center_x = rect_.x + rect_.width / 2;
            int bias = fabs(center_x - depthimage.cols/2);
            double rotation = 0.0;
            double last_rotation = 0.0;
            double now_rotation = 0.0;
            double diff_rotation;

            diff_rotation = fabs(last_rotation) - fabs(rotation);

            if (center_x < ERROR_OFFSET_X_LEFT1)
                now_rotation = max_angle_speed_;
            else if ((center_x > ERROR_OFFSET_X_LEFT1) && (center_x < ERROR_OFFSET_X_LEFT2))
                now_rotation = k_rotation_speed_ * bias * pid_angle.p + diff_rotation * pid_angle.d;
            else if ((center_x > ERROR_OFFSET_X_RIGHT1) && (center_x < ERROR_OFFSET_X_RIGHT2))
                now_rotation = -(k_rotation_speed_ * bias * pid_angle.p + diff_rotation * pid_angle.d);
            else if (center_x > ERROR_OFFSET_X_RIGHT2)
                now_rotation = -max_angle_speed_;
            else
                now_rotation = 0;

            last_rotation = rotation;
            rotation = now_rotation;

            cout << "linear_speed: " << now_speed << " rotation speed: " << now_rotation << endl;
            cout << "distance: " << distance <<endl;

            enable_get_depth = false;
            speed_cmd.linear.x = now_speed;
            speed_cmd.angular.z = now_rotation;

            if (kcf_init_)
                cout << "send twist" << endl;
                twist_pub_.publish(speed_cmd);
        }
    }

    void PeopleTrack::run()
    {
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}
