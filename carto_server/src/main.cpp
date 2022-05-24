/*
 * @Author: luog3 5337828+luog3@user.noreply.gitee.com
 * @Date: 2022-05-15 18:56:17
 * @LastEditors: luog3 5337828+luog3@user.noreply.gitee.com
 * @LastEditTime: 2022-05-15 19:13:01
 * @FilePath: /cat_ws/src/carto_server/src/main.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <iostream>
#include <ros/ros.h>

#include "cartographer_ros_msgs/score_client.h"

bool handle_function(cartographer_ros_msgs::score_client::Request& req, cartographer_ros_msgs::score_client::Response& res)
{
    //显示请求信息
    // ROS_INFO("Request from %s with age %d", req.name.c_str(), req.age);
    //处理请求，结果写入response
    float score = req.score;

    std::cout << "------------------socre = " << score << std::endl;

    
    //返回true,正确处理了请求
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "carto_server_node");

    ros::NodeHandle nh;

    ros::ServiceServer service = nh.advertiseService("pose_score_client", handle_function); //提供服务
    ros::spin();

    return 0;
}