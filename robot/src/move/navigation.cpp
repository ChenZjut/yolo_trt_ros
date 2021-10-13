/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/8/9 下午7:43
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/8/9 下午7:43
 * @Version 1.0
 */
#include <move/navigation.h>

namespace MoveNS
{
    Navigation::Navigation()
        : pnh_("~")
        , nh_()
    {
        initROS();
    }

    Navigation::~Navigation()
    {

    }

    void Navigation::initROS()
    {

        navi_sub_ = nh_.subscribe("navigation_pose", 10, &Navigation::goalCB, this);
        cmd_vel_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 5);

        try {
            tf_listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(1.0));
            tf_listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(1.0));
        }
        catch (const tf::TransformException& ex) {
            ROS_ERROR("%s", ex.what());
            return;
        }
    }

    void Navigation::goalCB(const geometry_msgs::PoseStamped msg)
    {
        pose = msg;
//        std::cout << " " << pose;
        move(pose, "map");
    }

    bool Navigation::move(geometry_msgs::PoseStamped pose, std::string frame_id)
    {

        // create the action client
        // "true" causes the client to spin its own thread
        // so don't need ros::spin()
        MoveBaseClient move_base("move_base", true);

        // wait for 20 seconds for the action server to become available
        move_base.waitForServer(ros::Duration(20));

        // Send a goal to move_base
        // target setting
        goal.target_pose = pose;
        goal.target_pose.header.frame_id = frame_id;
        goal.target_pose.header.stamp = ros::Time::now();

        ROS_INFO("send goal");

        move_base.sendGoal(goal);

        // Wait for the action to return
        bool finished_within_time = move_base.waitForResult(ros::Duration(40));

        if (!finished_within_time)
        {
            ROS_INFO("fail reach loc");
            move_base.cancelGoal();
            return false;
        }
        else
        {
            actionlib::SimpleClientGoalState state = move_base.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("goal reach");
                return true;
            }
            else
            {
                ROS_INFO("goal fail");
                return false;
            }
        }
    }

    void Navigation::run()
    {
        ros::Rate loop_rate(10);
        while(ros::ok())
        {
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

}