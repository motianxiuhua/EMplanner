/**
 * @file planning_node.cpp
 * @author xiuhua_liang (xiuhua_liang@163.com)
 * @brief // TODO 后续将定位和全局路径部分拿出来单独称为两个模块
 * @version 0.1
 * @date 2024-01-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#include "planning_node.h"

using namespace std;
namespace planning
{
    /**
     * @brief 规划默认构造函数用于加载规划参数
     * 
     */
    PlanningNode::PlanningNode()
    {
        perception = std::make_unique<PerceptionObstacle>(); // TODO 修改接口
        localization_info.is_odometry = false;

        ros::NodeHandle n("~"); // 句柄

        n.param<string>("role_name", role_name, "ego_vehicle");

        //参考线平滑参数
        n.param<double>("ref_weight_smooth", referenceline_params["ref_weight_smooth"], 70.0);
        n.param<double>("ref_weight_path_length", referenceline_params["ref_weight_path_length"], 10.0);
        n.param<double>("ref_weight_ref_deviation", referenceline_params["w_lat_offset"], 20.0);
        n.param<double>("x_lower_bound", referenceline_params["x_lower_bound"], -2.0);
        n.param<double>("x_upper_bound", referenceline_params["x_upper_bound"], 2.0);
        n.param<double>("y_lower_bound", referenceline_params["y_lower_bound"], -2.0);
        n.param<double>("y_upper_bound", referenceline_params["y_upper_bound"], 2.0);
        
        //emplanner参数
        //path DP 
        n.param<double>("dp_sample_l", emplanner_params["dp_sample_l"], 1.0);
        n.param<double>("dp_sample_s", emplanner_params["dp_sample_s"], 5.0);
        n.param<double>("dp_sample_rows", emplanner_params["dp_sample_rows"], 5);
        n.param<double>("dp_sample_cols", emplanner_params["dp_sample_cols"], 5);
        n.param<double>("dp_interp_ds", emplanner_params["dp_interp_ds"], 1);
        n.param<double>("dp_cost_collision", emplanner_params["dp_cost_collision"], 10e8);
        n.param<double>("dp_cost_dl", emplanner_params["dp_cost_dl"], 150);
        n.param<double>("dp_cost_ddl", emplanner_params["dp_cost_ddl"], 10);
        n.param<double>("dp_cost_dddl", emplanner_params["dp_cost_dddl"], 1);
        n.param<double>("dp_cost_ref", emplanner_params["dp_cost_ref"], 100);

        //path QP 
        n.param<double>("qp_cost_l", emplanner_params["qp_cost_l"], 15);
        n.param<double>("qp_cost_dl", emplanner_params["qp_cost_dl"], 1500);
        n.param<double>("qp_cost_ddl", emplanner_params["qp_cost_ddl"], 10);
        n.param<double>("qp_cost_dddl", emplanner_params["qp_cost_dddl"], 1);
        n.param<double>("qp_cost_centre", emplanner_params["qp_cost_centre"], 5);
        n.param<double>("qp_cost_end_l", emplanner_params["qp_cost_end_l"], 0);
        n.param<double>("qp_cost_end_dl", emplanner_params["qp_cost_end_dl"], 0);
        n.param<double>("qp_cost_end_ddl", emplanner_params["qp_cost_end_ddl"], 0);
        n.param<double>("qp_interp_ds", emplanner_params["qp_interp_ds"], 0.1);

        //speed DP
        n.param<double>("speed_dp_sample_row", emplanner_params["speed_dp_sample_row"], 60);
        n.param<double>("speed_dp_sample_col", emplanner_params["speed_dp_sample_col"], 16);
        n.param<double>("speed_dp_sample_s", emplanner_params["speed_dp_sample_s"], 0.5);
        n.param<double>("speed_dp_sample_t", emplanner_params["speed_dp_sample_t"], 0.5);
        
        n.param<double>("ref_speed", emplanner_params["ref_speed"], 10);
        n.param<double>("speed_dp_cost_ref_speed", emplanner_params["speed_dp_cost_ref_speed"], 100);
        n.param<double>("speed_dp_cost_accel", emplanner_params["speed_dp_cost_accel"], 10);
        n.param<double>("speed_dp_cost_obs", emplanner_params["speed_dp_cost_obs"], 1);
        n.param<double>("max_lateral_accel", emplanner_params["max_lateral_accel"], 0.2);
        //speed QP
        n.param<double>("speed_qp_cost_v_ref", emplanner_params["speed_qp_cost_v_ref"], 8);
        n.param<double>("speed_qp_cost_dds_dt", emplanner_params["speed_qp_cost_dds_dt"], 5);
        n.param<double>("speed_qp_cost_jerk", emplanner_params["speed_qp_cost_jerk"], 0);
        n.param<double>("speed_qp_interp_dt", emplanner_params["speed_qp_interp_dt"], 0.1);
        
         // 订阅carla消息
        location_subscriber = n.subscribe("/carla/" + role_name + "/odometry", 1, &PlanningNode::callbackCarlaOdom, this);
        global_path_subscriber = n.subscribe("/carla/" + role_name + "/waypoints", 1, &PlanningNode::callbackGlobalPath, this);
        imu_subscriber = n.subscribe("/carla/" + role_name + "/imu", 1, &PlanningNode::callbackIMU, this);
        detected_objects_subscriber = n.subscribe("/carla/" + role_name + "/objects", 1, &PlanningNode::callbackDetectedObjects, this);

        // //发送轨迹给控制
        local_waypoints_pub = n.advertise<local_waypoint_msgs::LocalWaypointArray>("/planning/local_waypoint", 10);
        trajectory_pub = n.advertise<nav_msgs::Path>("/planning/final_path", 10);
        history_paths_pub = n.advertise<nav_msgs::Path>("/planning/history_paths", 10);
        speed_marker_pub = n.advertise<visualization_msgs::Marker>("/speed_marker_text", 10);
        point_marker_pub = n.advertise<visualization_msgs::Marker>("/point_marker", 10);
        
    }

    /**
     * @brief 主循环
     *
     */
    void PlanningNode::MainLoop()
    {
        ros::Rate rate(10.0);

        ReferenceLine reference_line;     //当前参考线
        ReferenceLine pre_reference_line; //上一周期参考线

        Trajectory cur_trajectory; //当前轨迹
        Trajectory trajectory; //当前轨迹
        Trajectory pre_trajectory; //上一周期轨迹
        std::vector<Trajectory> history_trajectory; //历史轨迹

        ros::NodeHandle nh("~"); // 句柄
        ros::Publisher planning_path_pub = 
            nh.advertise<nav_msgs::Path>("/planning/planning_path", 10); // 路径规划可视化
        ros::Publisher dp_planning_path_pub = 
            nh.advertise<nav_msgs::Path>("/planning/dp_path", 10); // 路径规划可视化
        ros::Publisher qp_planning_pub = 
            nh.advertise<nav_msgs::Path>("/planning/qp", 10); // 路径规划可视化
        ros::Publisher qp_planning_path_pub = 
            nh.advertise<nav_msgs::Path>("/planning/qp_path", 10); // 路径规划可视化

        while (ros::ok())
        {
            //获取定位信息和全局路径之后开始规划
             if(!routing_path_points.empty() && localization_info.is_odometry){
                // 2.参考线生成,参考新默认-30m~150m
                std::unique_ptr<ReferenceLineProvider> reference_line_provider = 
                    std::make_unique<ReferenceLineProvider>();
                pre_reference_line = reference_line;
                //传参应该是数据类型，而不是类的对象
                 reference_line_provider->Provide(routing_path_points, localization_info,
                                                pre_reference_line, reference_line, referenceline_params);

                // 3.规划器规划
                std::unique_ptr<EMPlanner> em_planner =
                    std::make_unique<EMPlanner>(emplanner_params); //规划器初始化
                TrajectoryPoint plan_start;
                Trajectory stitch_trajectory;
                pre_trajectory = trajectory;
                em_planner->CalPlaningStartPoint(pre_trajectory, localization_info,
                                                &plan_start, &stitch_trajectory);
                std::vector<ReferencePoint> xy_virtual_obstacles;
                //接口和参数都对不上
                std::pair<std::unique_ptr<PathTimeGraph>,std::unique_ptr<SpeedTimeGraph>> planner = 
                em_planner->Plan(plan_start, reference_line, localization_info,
                                perception->static_obstacles(), perception->dynamic_obstacles(),
                                &cur_trajectory, xy_virtual_obstacles);
                // perception->UpdateVirtualObstacle(xy_virtual_obstacles);
                em_planner->StitchTrajectory(cur_trajectory, stitch_trajectory, trajectory);
                history_trajectory.push_back(trajectory);

                //规划起点可视化
                plan_start_visualization(plan_start);

                //路径规划可视化
                planning_visualization(planner.first->planning_path().reference_points(), planning_path_pub);

                //动态规划
                planning_visualization(planner.first->dp_path_points_dense_cartersian(), dp_planning_path_pub);

                //二次规划
                planning_visualization(planner.first->qp_path_points_cartersian(), qp_planning_pub);
                planning_visualization(planner.first->qp_path_points_dense_cartersian(), qp_planning_path_pub);

                //规划轨迹可视化
                trajectory_visualization(trajectory.trajectory_points());

                //显示障碍物信息
                detected_object_visualization(perception->get_obstacles());

                //历史规划可视化
                history_trajectory_visualization(history_trajectory);
            }
            else{
                trajectory.clear_trajectory_points();
            }
            //发布局部路径给控制
            local_waypoint_msgs::LocalWaypointArray trajectory_msg_array;
            std::vector<TrajectoryPoint> trajectory_msg;
            trajectory_msg = trajectory.trajectory_points();
            for(int i = 0; i < trajectory_msg.size(); i++)
            {
                local_waypoint_msgs::LocalWaypoint point;
                point.header.stamp =  ros::Time().fromSec(trajectory_msg[i].t);
                point.header.frame_id = "map";
                point.pose.position.x = trajectory_msg[i].x;
                point.pose.position.y = trajectory_msg[i].y;
                geometry_msgs::Quaternion quat =
                tf::createQuaternionMsgFromYaw(trajectory_msg[i].heading);
                point.pose.orientation = quat;
                point.velocity = trajectory_msg[i].v;
                point.acceleration =  trajectory_msg[i].a;
                point.kappa = trajectory_msg[i].kappa;
                trajectory_msg_array.waypoints.push_back(point);
            }
            local_waypoints_pub.publish(trajectory_msg_array);
            
            //1. 调用回调函数得到定位感知全局路径信息
            ros::spinOnce();
            rate.sleep();
        }

    }

    /**
     * @brief carla回调函数 1. 定位信息 2. 全局路径 3. 障碍物感知信息
     * 
     */

    /**
     * @brief carla里程计信息的回调函数, 根据里程计信息获取车辆当前的位置
     * @brief 利用carla的信息充当定位模块
     * @param msg  nav_msgs/Odometry
     */
    void PlanningNode::callbackCarlaOdom(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // 坐标转换
        geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
        tf::Quaternion quat;
        tf::quaternionMsgToTF(odom_quat, quat);

        // 根据转换后的四元数，获取roll pitch yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        localization_info.x = msg->pose.pose.position.x;
        localization_info.y = msg->pose.pose.position.y;
        localization_info.heading = yaw;//TODO yaw是车身与世界坐标系X之间的夹角，而heading是速度与X之间的夹角，两者之间差一个侧偏角

        localization_info.vx = msg->twist.twist.linear.x;
        localization_info.vy = msg->twist.twist.linear.y;
        localization_info.v = sqrt(pow(localization_info.vx, 2) + pow(localization_info.vy, 2));

        localization_info.t = msg->header.stamp.toSec();
        localization_info.is_odometry = true;
        // ROS_INFO("plan time %f",cur_pose.t);
    }

    /**
     * @brief 获取IMU信息(求加速度)
     *
     * @param msg
     */
    void PlanningNode::callbackIMU(const sensor_msgs::Imu::ConstPtr &msg)
    {
        // cur_pose.angular_velocity =
        //     msg->angular_velocity.z; // 平面角速度(绕z轴转动的角速度)
        localization_info.ax = msg->linear_acceleration.x;
        localization_info.ay = msg->linear_acceleration.y;
        localization_info.a = sqrt(localization_info.ax * localization_info.ax +
                          localization_info.ay * localization_info.ay); // 加速度
    }

    /**
     * @brief 处理获取的全局路径
     *
     * @param msg carla的全局路径
     */
    void PlanningNode::callbackGlobalPath(const nav_msgs::Path::ConstPtr &msg)
    {
        routing_path_points.clear();
        MapPoint global_path_point; //TODO 全局路径可以获得的信息 z轴是否有用？ 无用
        // 将path信息转换到global_path中
        for (int i = 0; i < msg->poses.size(); i++)
        {
            global_path_point.x = msg->poses[i].pose.position.x;
            global_path_point.y = msg->poses[i].pose.position.y;
            // 坐标转换
            geometry_msgs::Quaternion odom_quat = msg->poses[i].pose.orientation;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(odom_quat, quat);
            // 根据转换后的四元数，获取roll pitch yaw
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            global_path_point.heading = yaw;

            if (routing_path_points.size() == 0)
            {
                routing_path_points.push_back(global_path_point);
            }
            // 防止路径点重合
            else if ((routing_path_points.size() > 0) &&
                     abs(global_path_point.x - routing_path_points[routing_path_points.size() - 1].x) >
                         0.001 &&
                     abs(global_path_point.y - routing_path_points[routing_path_points.size() - 1].y) >
                         0.001)
            {
                routing_path_points.push_back(global_path_point);
                // ROS_INFO("global_x:%.f,global_y:%.f",path_point.x,path_point.y);
            }
        }
        // ROS_INFO("Received global_path successfully, The size of global is :%zu", global_path.size());
    }

    /**
     * @brief 获取探测到的目标信息
     * 信息详细说明：https://docs.ros.org/en/melodic/api/derived_object_msgs/html/msg/ObjectArray.html
     * @param msg
     */
    void PlanningNode::callbackDetectedObjects(
        const derived_object_msgs::ObjectArray::ConstPtr &msg)
    {
        // ROS_INFO("Received dectected objects successfully ...");
        std::vector<ObstacleInfo> detected_objects;

        for (int i = 0; i < msg->objects.size(); i++)
        {
            ObstacleInfo obs; //TODO 修改数据类型

            obs.center_point.x = msg->objects[i].pose.position.x;
            obs.center_point.y = msg->objects[i].pose.position.y;
            obs.center_point.vx = msg->objects[i].twist.linear.x;
            obs.center_point.vy = msg->objects[i].twist.linear.y;
            obs.center_point.v = sqrt(pow(obs.center_point.vx, 2) + pow(obs.center_point.vy, 2));
            // ROS_INFO("leader car speedvx: %.2f , position,vy:%.2f,v:%.2f",object.vx,object.vy,object.v);
            // 坐标转换
            geometry_msgs::Quaternion ob_quat = msg->objects[i].pose.orientation;
            tf::Quaternion quat;
            tf::quaternionMsgToTF(ob_quat, quat);
            // 根据转换后的四元数，获取roll pitch yaw
            double roll, pitch, yaw;
            tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
            obs.center_point.heading = yaw;

            obs.center_point.ax = msg->objects[i].accel.linear.x;
            obs.center_point.ay = msg->objects[i].accel.linear.y;
            obs.center_point.a =  sqrt(pow(obs.center_point.ax, 2) + pow(obs.center_point.ay, 2));

            obs.x_radius = msg->objects[i].shape.dimensions[0] / 2.0;
            obs.y_radius = msg->objects[i].shape.dimensions[1] / 2.0;
            obs.ID = msg->objects[i].id;

            detected_objects.push_back(obs);
        }
        perception->set_obstacles(detected_objects);
    }


    /**
     * @brief 
     * 
     * @param trajectory 
     */
    void PlanningNode::trajectory_visualization(const std::vector<TrajectoryPoint> &trajectory){
        nav_msgs::Path best_path;
        best_path.header.frame_id = "map";
        for (int i = 0; i < trajectory.size(); i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = trajectory[i].x;
            pose.pose.position.y = trajectory[i].y;

            geometry_msgs::Quaternion quat =
                tf::createQuaternionMsgFromYaw(trajectory[i].heading);
            pose.pose.orientation = quat;
            // pose.header.stamp =  ros::Time().fromSec(trajectory[i].t);
            best_path.poses.push_back(pose);
        }
        trajectory_pub.publish(best_path);
    }

    void PlanningNode::planning_visualization(const std::vector<ReferencePoint>& planning_path, const ros::Publisher &path_pub){
        nav_msgs::Path best_path;
        best_path.header.frame_id = "map";
        for (int i = 0; i < planning_path.size(); i++)
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = planning_path[i].x;
            pose.pose.position.y = planning_path[i].y;

            geometry_msgs::Quaternion quat =
                tf::createQuaternionMsgFromYaw(planning_path[i].heading);
            pose.pose.orientation = quat;
            // pose.header.stamp =  ros::Time().fromSec(trajectory[i].t);
            best_path.poses.push_back(pose);
        }
        path_pub.publish(best_path);
    }

    /**
     * @brief 
     * 
     * @param obstacle 
     */
    void PlanningNode::detected_object_visualization(const std::vector<ObstacleInfo> &obstacle){
        int id_ = 0;
        for (auto &object : obstacle)
        {
            visualization_msgs::Marker speed_marker;
            speed_marker.header.frame_id = "map";
            speed_marker.header.stamp = ros::Time::now();
            speed_marker.ns = "planning/speed_marker";
            speed_marker.action = visualization_msgs::Marker::ADD;
            speed_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

            speed_marker.pose.orientation.w = 1.0;
            speed_marker.id = id_++;

            speed_marker.scale.x = 1.5;
            speed_marker.scale.y = 1.5;
            speed_marker.scale.z = 1.5;

            speed_marker.color.b = 0;
            speed_marker.color.g = 0;
            speed_marker.color.r = 255;
            speed_marker.color.a = 1;

            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << object.center_point.v; // 设置精度为2位

            speed_marker.text = ss.str() + "m/s";

            speed_marker.pose.position.x = object.center_point.x;
            speed_marker.pose.position.y = object.center_point.y;

            speed_marker_pub.publish(speed_marker);
        }
    }
    void PlanningNode::history_trajectory_visualization(const std::vector<Trajectory> &history_trajectory){
        nav_msgs::Path history_path;
        history_path.header.frame_id = "map";
        history_path.header.stamp = ros::Time::now();
        for (auto history : history_trajectory)
        {
            int count = 0;
            // TODO每次只绘制缓存中最后一次的轨迹为什么呢？
            std::vector<TrajectoryPoint> trajectory = history.trajectory_points();
            for (int i = 0; i < trajectory.size() && count < 1; i++, count++) 
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = trajectory[i].x;
                pose.pose.position.y = trajectory[i].y;
                // pose.header.stamp =  ros::Time().fromSec(trajectory[i].t);
                geometry_msgs::Quaternion quat =
                    tf::createQuaternionMsgFromYaw(trajectory[i].heading);
                pose.pose.orientation = quat;
                history_path.poses.push_back(pose);
            }
            history_paths_pub.publish(history_path);
        }
    }

    /**
     * @brief 用于显示规划起点
     *
     * @param plan_start
     */
    void PlanningNode::plan_start_visualization(const TrajectoryPoint &plan_start)
    {
        visualization_msgs::Marker point_marker;
        point_marker.header.frame_id = "map";
        point_marker.header.stamp = ros::Time::now();
        point_marker.ns = "planning/point_marker";
        point_marker.action = visualization_msgs::Marker::ADD;
        point_marker.type = visualization_msgs::Marker::SPHERE;

        point_marker.pose.orientation.w = 1.0;

        point_marker.scale.x = 0.5;
        point_marker.scale.y = 0.5;
        point_marker.scale.z = 0.5;

        point_marker.color.b = 0;
        point_marker.color.g = 0;
        point_marker.color.r = 255;
        point_marker.color.a = 1;

        point_marker.pose.position.x = plan_start.x;
        point_marker.pose.position.y = plan_start.y;

        point_marker_pub.publish(point_marker);
    }
}   