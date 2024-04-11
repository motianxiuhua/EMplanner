/**
 * @file planning_node.h
 * @author xiuhua_liang (xiuhua_liang@163.com)
 * @brief 
 * @version 0.1
 * @date 2024-01-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

#include <iostream>
#include <sstream>
#include <iomanip>
#include <unordered_map>
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <derived_object_msgs/Object.h>
#include <derived_object_msgs/ObjectArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <local_waypoint_msgs/LocalWaypoint.h> 
#include <local_waypoint_msgs/LocalWaypointArray.h> 

#include "pnc_point.h"
#include "perception_obstacle.h"
#include "emplanner/trajectory.h"
#include "emplanner/EMPlanner.h"
#include "reference_line.h"

namespace planning
{
    class PlanningNode
    {
    public:
        PlanningNode();
        void MainLoop();

        /*ros订阅回调函数*/
        void callbackCarlaOdom(const nav_msgs::Odometry::ConstPtr &msg);
        void callbackIMU(const sensor_msgs::Imu::ConstPtr &msg);
        void callbackGlobalPath(const nav_msgs::Path::ConstPtr &msg);
        void callbackDetectedObjects(const derived_object_msgs::ObjectArray::ConstPtr &msg);

        /*rviz可视化函数*/
        void trajectory_visualization(const std::vector<TrajectoryPoint> &trajectory);
        void detected_object_visualization(const std::vector<ObstacleInfo> &obstacle);
        void history_trajectory_visualization(const std::vector<Trajectory> &history_trajectory);
        void plan_start_visualization(const TrajectoryPoint &plan_start);
        void planning_visualization(const std::vector<ReferencePoint>& planning_path, const ros::Publisher &path_pub);

    private:
        LocalizationInfo localization_info; //记录carla ros bridge里程计的定位信息
        std::vector<MapPoint> routing_path_points; //记录carla ros bridge的全局路径信息
        std::unique_ptr<PerceptionObstacle> perception;
        std::unordered_map<std::string, double> referenceline_params;
        std::unordered_map<std::string, double> emplanner_params;
        std::string role_name;

        ros::Subscriber location_subscriber;
        ros::Subscriber global_path_subscriber;
        ros::Subscriber imu_subscriber;
        ros::Subscriber detected_objects_subscriber;

        ros::Publisher local_waypoints_pub;
        ros::Publisher trajectory_pub;      // 发布最优轨迹，用于rviz可视化
        ros::Publisher history_paths_pub;   // 发布历史参考路径，用于rviz可视化
        ros::Publisher speed_marker_pub; // 发布目标速度，用于rviz可视化
        ros::Publisher point_marker_pub; // 规划起点可视化
    };
}