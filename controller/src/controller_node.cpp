/**
 * @file controller_node.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-06-07
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "controller_node.h"

using namespace std;

namespace controller
{
  ControllerNode::ControllerNode()
  {
    ros::NodeHandle n("~"); // 句柄
    n.param<string>("role_name", role_name, "ego_vehicle");

    // 纵向PID参数
    n.param<double>("kp", kp, 0.85);
    n.param<double>("ki", ki, 0.02);
    n.param<double>("kd", kd, 0.1);

    // LQR_dynamics Q  R矩阵权重
    n.param<double>("Q_ed", Q_ed, 67.0);
    n.param<double>("Q_ed_dot", Q_ed_dot, 1.0);
    n.param<double>("Q_ephi", Q_ephi, 70.0);
    n.param<double>("Q_ephi_dot", Q_ephi_dot, 1.0);
    n.param<double>("R_value", R_value, 35.0);

    // setup subscriber
    cur_pose_sub = n.subscribe("/carla/" + role_name + "/odometry", 10, &ControllerNode::callbackCarlaOdom, this);
    local_path_sub = n.subscribe("/planning/local_waypoint", 10, &ControllerNode::callbackLocalPath, this);

    // setup publishers
    path_pub = n.advertise<nav_msgs::Path>("/trajectory", 10);
    control_cmd_pub = n.advertise<carla_msgs::CarlaEgoVehicleControl>(
        "/carla/" + role_name + "/vehicle_control_cmd", 10);
  }

  /**
   * @brief 主循环
   *
   */
  void ControllerNode::MainLoop()
  {
    ros::Rate rate(50);//控制每隔1/50(0.02s)更新一次

    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
   
    Q.setZero(4, 4);
    Q(0, 0) = Q_ed;
    Q(1, 1) = Q_ed_dot;
    Q(2, 2) = Q_ephi;
    Q(3, 3) = Q_ephi_dot;

    // R矩阵
    R.setZero(1, 1);
    R(0, 0) = R_value;

    controller::Controller control(
                             Q, R,
                             kp, ki, kd
                             );
    //计算LQR反馈率K
    control.CreateLqrOffline(Q, R); // 离线LQR
    vector<double> cmd(3);

    while (ros::ok())
    {
      // ROS_INFO("start Inteartion");
      // double current_timestamp = ros::Time::now().toSec();

      /***********************************控制主体**************************************/
      if (local_waypoints.size() > 1)
      {
        control.ComputeControlValue(local_waypoints, cur_pose); //计算期望控制值
        // TODO 精确的应该使用位置速度双PID并且制作对应的油门速度标定表，但是现在制作这个表需要试验，之后可以作为优化的点
        control.LongitudinalControl(); //速度PID
        control.LateralControl(); //动力学LQR
        cmd = control.get_command();
      }
      else
      {
        cmd = {0, 0, 1.0};
      }

      ROS_INFO("throttle: %2f, steer: %2f, brake: %2f", cmd[0], cmd[1], cmd[2]);

      /***********************************发布控制指令**************************************/
      carla_msgs::CarlaEgoVehicleControl control_cmd;
      control_cmd.throttle = cmd[0];
      control_cmd.steer = cmd[1];
      control_cmd.brake = cmd[2];
      control_cmd.hand_brake = false;
      control_cmd.reverse = false;
      control_cmd_pub.publish(control_cmd);

      ros::spinOnce();
      rate.sleep();

      // ROS_INFO("The iteration end.");
    }
  }

  /**
   * @brief carla里程计信息的回调函数, 根据里程计信息获取车辆当前的位置，并发布行驶路径
   *
   * @param msg
   */
  void ControllerNode::callbackCarlaOdom(const nav_msgs::Odometry::ConstPtr &msg)
  {
    // 坐标转换
    geometry_msgs::Quaternion odom_quat = msg->pose.pose.orientation;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odom_quat, quat);

    // 根据转换后的四元数，获取roll pitch yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    cur_pose.x = msg->pose.pose.position.x;
    cur_pose.y = msg->pose.pose.position.y;
    cur_pose.heading = yaw;

    cur_pose.vx = msg->twist.twist.linear.x;
    cur_pose.vy = msg->twist.twist.linear.y;
    cur_pose.v = std::sqrt(cur_pose.vx * cur_pose.vx + cur_pose.vy * cur_pose.vy);

    cur_pose.t = msg->header.stamp.toSec();

    // 将Odometry转换为Path，发布车辆行驶路径
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = msg->pose.pose.position.x;
    this_pose_stamped.pose.position.y = msg->pose.pose.position.y;
    this_pose_stamped.pose.position.z = msg->pose.pose.position.z;

    this_pose_stamped.pose.orientation.x = msg->pose.pose.orientation.x;
    this_pose_stamped.pose.orientation.y = msg->pose.pose.orientation.y;
    this_pose_stamped.pose.orientation.z = msg->pose.pose.orientation.z;
    this_pose_stamped.pose.orientation.w = msg->pose.pose.orientation.w;

    this_pose_stamped.header.stamp = ros::Time::now();
    this_pose_stamped.header.frame_id = "map";
    path_msg.poses.push_back(this_pose_stamped);

    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "map";

    path_pub.publish(path_msg);
  }

  /**
   * @brief 获取局部规划路径
   *
   * @param msg
   */
  void ControllerNode::callbackLocalPath(const local_waypoint_msgs::LocalWaypointArray::ConstPtr &msg)
  {
    // ROS_INFO("Received final waypoints in trajectory controller ...");
    local_waypoints.resize(msg->waypoints.size());
    for (int i = 0; i < msg->waypoints.size(); i++)
    {
      TrajectoryPoint tre_point;
      tre_point.x = msg->waypoints[i].pose.position.x;
      tre_point.y = msg->waypoints[i].pose.position.y;
      tre_point.v = msg->waypoints[i].velocity;
      tre_point.a = msg->waypoints[i].acceleration;
      tre_point.t = msg->waypoints[i].header.stamp.toSec();
      tre_point.kappa = msg->waypoints[i].kappa;
      geometry_msgs::Quaternion odom_quat = msg->waypoints[i].pose.orientation;
      tf::Quaternion quat;
      tf::quaternionMsgToTF(odom_quat, quat);
      double roll, pitch, yaw;
      tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
      tre_point.heading = yaw;
      local_waypoints[i] = tre_point;
    }
  }

} // controller