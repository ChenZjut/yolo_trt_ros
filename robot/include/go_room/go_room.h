/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/5 下午2:56
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/5 下午2:56
 * @Version 1.0
 */
#ifndef SRC_GO_ROOM_H
#define SRC_GO_ROOM_H

#include <stdlib.h>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

#include <people_tracking/people_tracking.h>


using namespace std;

namespace GoRoomNS
{
class GoRoom
{
private:
    // tracking
    PeopleTrackNS::PeopleTrack peopleTrack;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Subscriber asr_sub_;
    ros::Subscriber class_sub_;

    ros::Publisher tts_pub_;
    ros::Publisher navi_pub_;
    ros::Publisher end_pub_;

    // position
    float init_position_x_;
    float init_position_y_;
    float init_position_z_;
    float init_position_w_;

    float bedroom_x_;
    float bedroom_y_;
    float bedroom_z_;
    float bedroom_w_;

    float dining_x_;
    float dining_y_;
    float dining_z_;
    float dining_w_;

    float kitchen_x_;
    float kitchen_y_;
    float kitchen_z_;
    float kitchen_w_;

    geometry_msgs::PoseStamped pose;
    std_msgs::Bool end_msg_;
    std_msgs::String class_;

    // initROS
    void initROS();

    void asr_cb(const std_msgs::String msg);
    void class_cb(const std_msgs::StringConstPtr& msg);

    void speak(string str);

public:
    GoRoom();
    ~GoRoom();
    void run();
};

}


#endif //SRC_GO_ROOM_H
