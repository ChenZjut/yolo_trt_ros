/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/18 下午3:45
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/18 下午3:45
 * @Version 1.0
 */
#include <communication/communication.h>

namespace CommunicationNS
{
    Communication::Communication()
        : nh_()
        , pnh_("~")
    {
        initROS();
    }

    Communication::~Communication()
    {

    }

    void Communication::initROS()
    {
        pnh_.param<int>("continue_time", continue_time, 10);

        asr_pub_ = nh_.advertise<std_msgs::String>("asr_result", 1);
    }

    void Communication::get_asr()
    {
        ASR_Client xf_asr("/xf_asr/home_recognize", true);
        xf_asr.waitForServer(ros::Duration(3));

        xf_ros::HomeRecognizeGoal homeRecognizeGoal;
        std_msgs::String msg;

        msg.data = "home";
        homeRecognizeGoal.bnf_name = msg;
        homeRecognizeGoal.continue_time = continue_time;

        xf_asr.sendGoal(homeRecognizeGoal);

        bool finished_within_time = xf_asr.waitForResult(ros::Duration(10));

        if (!finished_within_time)
        {
            ROS_INFO("Start again");
            return;
        }

        std_msgs::String str = xf_asr.getResult()->msg;

        cout << " " << str.data << endl;

        asr_pub_.publish(str);
    }

    void Communication::run()
    {
        ros::Rate loop_rate(10);
        while (ros::ok())
        {
            get_asr();
            loop_rate.sleep();
        }
    }
}