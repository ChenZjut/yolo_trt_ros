/*
 * @Description: 
 * @Author: ubuntu
 * @Date: 2021/8/9 下午9:29
 * @LastEditors: ubuntu
 * @LastEditTime: 2021/8/9 下午9:29
 * @Version 1.0
 */

#include <move/navigation.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "navigation_node");
    MoveNS::Navigation app;
    app.run();
    return 0;
}