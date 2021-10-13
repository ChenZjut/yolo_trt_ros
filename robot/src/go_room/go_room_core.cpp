/*
 * @Description: Go to room to find the trash.
 * @Author: ubuntu
 * @Date: 2021/9/5 下午2:56
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/5 下午2:56
 * @Version 1.0
 */

#include <go_room//go_room.h>

namespace GoRoomNS
{
    GoRoom::GoRoom()
        : nh_()
        , pnh_("~")
    {
        initROS();
    }

    GoRoom::~GoRoom()
    {

    }

    void GoRoom::initROS()
    {
        // init Robot
        pnh_.param<float>("init_position_x", init_position_x_, 0.0);
        pnh_.param<float>("init_position_y", init_position_y_, 0.0);
        pnh_.param<float>("init_position_z", init_position_z_, 0.0);
        pnh_.param<float>("init_position_w", init_position_w_, 0.0);

        // bedroom position
        pnh_.param<float>("bedroom_x", bedroom_x_, 0.0);
        pnh_.param<float>("bedroom_y", bedroom_y_, 0.0);
        pnh_.param<float>("bedroom_z", bedroom_z_, 0.0);
        pnh_.param<float>("bedroom_w", bedroom_w_, 0.0);

        // dining room position
        pnh_.param<float>("dining_x", dining_x_, 0.0);
        pnh_.param<float>("dining_y", dining_y_, 0.0);
        pnh_.param<float>("dining_z", dining_z_, 0.0);
        pnh_.param<float>("dining_w", dining_w_, 0.0);

        // kitchen position
        pnh_.param<float>("kitchen_x", kitchen_x_, 0.0);
        pnh_.param<float>("kitchen_y", kitchen_y_, 0.0);
        pnh_.param<float>("kitchen_z", kitchen_z_, 0.0);
        pnh_.param<float>("kitchen_w", kitchen_w_, 0.0);

        asr_sub_ = nh_.subscribe("asr_result", 10, &GoRoom::asr_cb, this);
        class_sub_ = nh_.subscribe("object_class", 10, &GoRoom::class_cb, this);

        tts_pub_ = nh_.advertise<std_msgs::String>("/xf/tts/words", 1);
        navi_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("navigation_pose", 1);
        end_pub_ = nh_.advertise<std_msgs::Bool>("tracking_end", 1);
    }

    void GoRoom::speak(string str)
    {
        std_msgs::String msg;
        msg.data = str;
        tts_pub_.publish(msg);
    }

    void GoRoom::class_cb(const std_msgs::StringConstPtr &msg)
    {
        class_ = *msg;
    }

    void GoRoom::asr_cb(const std_msgs::String msg)
    {
        string str = msg.data;
        for (auto& x: str)
        {
            x = tolower(x);
        }

        if (str.find("start") < str.size())
        {
            pose.header.frame_id = "map";
            pose.pose.position.x = init_position_x_;
            pose.pose.position.y = init_position_y_;
            pose.pose.position.z = init_position_z_;
            pose.pose.orientation.w = init_position_w_;
            navi_pub_.publish(pose);

            speak("Where should I go");
        }

        if (str.find("bed") < str.size())
        {
            pose.header.frame_id = "map";
            pose.pose.position.x = bedroom_x_;
            pose.pose.position.y = bedroom_y_;
            pose.pose.position.z = bedroom_z_;
            pose.pose.orientation.w = bedroom_w_;
            navi_pub_.publish(pose);

            speak("Goal Reach");
        }

        if (str.find("dining") < str.size())
        {
            pose.header.frame_id = "map";
            pose.pose.position.x = dining_x_;
            pose.pose.position.y = dining_y_;
            pose.pose.position.z = dining_z_;
            pose.pose.orientation.w = dining_w_;
            navi_pub_.publish(pose);

            speak("Goal Reach");
        }

        if (str.find("kitchen") < str.size())
        {
            pose.header.frame_id = "map";
            pose.pose.position.x = kitchen_x_;
            pose.pose.position.y = kitchen_y_;
            pose.pose.position.z = kitchen_z_;
            pose.pose.orientation.w = kitchen_w_;
            navi_pub_.publish(pose);

            speak("Goal Reach");
        }

        if (str.find("follow") < str.size())
        {
            end_msg_.data = false;
            speak("start follow");
            ros::Duration(1);
            end_pub_.publish(end_msg_);
        }

        if (str.find("search") < str.size())
        {
            end_msg_.data = true;
            end_pub_.publish(end_msg_);
            speak("start search");
            ros::Duration(10);

            if (!class_.data.empty() && (class_.data != "people"))
            {
                speak("I find" + class_.data);
            }

        }

        // stop the robot
        if (str.find("stop") < str.size())
        {
            end_msg_.data = true;
            end_pub_.publish(end_msg_);
            speak("stop");
        }
    }

    void GoRoom::run()
    {
        ros::Rate loop_rate(10);

        while (ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}
