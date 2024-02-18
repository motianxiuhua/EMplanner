/**
 * @file planning_main.cpp
 * @author xiuhua_liang (xiuhua_liang@163.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include <iostream>

#include "planning_node.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planning");
    
    planning::PlanningNode planning_node;
    
    ROS_INFO("Start Planning");
    
    planning_node.MainLoop();

    return 0;
}