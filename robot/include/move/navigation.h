/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/8/9 下午6:59
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/8/9 下午6:59
 * @Version 1.0
 */
#ifndef SRC_NAVIGATION_H
#define SRC_NAVIGATION_H

#include <iostream>
#include <cmath>
#include <string>
#include <ctime>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_msgs/GoalStatus.h>

#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <move_base_msgs//MoveBaseGoal.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

namespace MoveNS
{
class Navigation
{
private:
    // move_base_msgs::MoveBaseAction: move_base 's target in the world
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber navi_sub_;

    ros::Publisher cmd_vel_pub;

    tf::TransformListener tf_listener;

    move_base_msgs::MoveBaseGoal goal;
    geometry_msgs::PoseStamped pose;

    // callback
    void goalCB(const geometry_msgs::PoseStamped msg);

    // function
    void initROS();
public:
    Navigation();
    ~Navigation();
    void run();
    bool move(geometry_msgs::PoseStamped pose, std::string frame_id="map");
};
}
#endif //SRC_NAVIGATION_H
