/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/18 下午3:46
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/18 下午3:46
 * @Version 1.0
 */

#include <communication/communication.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "communication_node");
    CommunicationNS::Communication app;
    app.run();
    ros::spin();
    return 0;
}