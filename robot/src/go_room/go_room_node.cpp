/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/9/5 下午2:57
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/9/5 下午2:57
 * @Version 1.0
 */

#include <go_room//go_room.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "go_room_node");
    GoRoomNS::GoRoom app;
    app.run();
    ros::spin();
    return 0;
}