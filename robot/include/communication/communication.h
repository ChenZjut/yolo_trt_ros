/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/18 下午3:36
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/18 下午3:36
 * @Version 1.0
 */
#ifndef SRC_COMMUNICATION_H
#define SRC_COMMUNICATION_H

#include <iostream>

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <xf_ros/HomeRecognizeAction.h>
#include <xf_ros/TTSAction.h>
#include <actionlib/client/simple_action_client.h>

using  namespace std;
namespace CommunicationNS
{
class Communication
{
private:
    typedef actionlib::SimpleActionClient<xf_ros::HomeRecognizeAction> ASR_Client;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;

    ros::Publisher asr_pub_;

    int continue_time;

    void initROS();

public:
    Communication();
    ~Communication();
    void get_asr();
    void run();
};
}

#endif //SRC_COMMUNICATION_H
