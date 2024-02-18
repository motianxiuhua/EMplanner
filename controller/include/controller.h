/**
 * @file controller.h
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-06-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <cfloat>
#include <cmath>
#include <cstdlib>
#include <deque>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include "ros/ros.h"
#include <Eigen/Eigen>

#include "pnc_point.h"
#include <local_waypoint_msgs/LocalWaypoint.h> 
#include <local_waypoint_msgs/LocalWaypointArray.h> 
namespace controller
{
  class Controller
  {
  public:
    /***********************************整车参数**************************************/
    double L = 3.0;                                                       // 轴距
    double cf = -155494.663;                                                // 前轮侧偏刚度,左右轮之和
    double cr = -155494.663;                                                // 后轮侧偏刚度, 左右轮之和
    double mass_fl = 1845.0 / 4;                                            // 左前悬的质量
    double mass_fr = 1845.0 / 4;                                            // 右前悬的质量
    double mass_rl = 1845.0 / 4;                                            // 左后悬的质量
    double mass_rr = 1845.0 / 4;                                            // 右后悬的质量
    double mass_front = mass_fl + mass_fr;                                  // 前悬质量
    double mass_rear = mass_rl + mass_rr;                                   // 后悬质量
    double mass = mass_front + mass_rear;                                   // 车辆载荷
    double lf = L * (1.0 - mass_front / mass);                              // 汽车前轮到中心点的距离
    double lr = L * (1.0 - mass_rear / mass);                               // 汽车后轮到中心点的距离
    double Iz = std::pow(lf, 2) * mass_front + std::pow(lr, 2) * mass_rear; // 车辆绕z轴转动的转动惯量
    double max_degree = 70 * M_PI / 180; // 最大前轮转向角(度)(carla中是最大转角是1.22对应70°)
    Controller() = default;
    Controller(
               const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
               const double &kp, const double &ki, const double &kd
              );
    double NormalizeAngle(double &angle);
    Eigen::MatrixXd CalcDlqr(double vx, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R);
    void CreateLqrOffline(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R);
    void ComputeControlValue(const std::vector<TrajectoryPoint> &local_waypoints, const TrajectoryPoint &cur_pose);
    Eigen::Matrix<double, 4, 1> CalStateError();
    double CalForwardAngle(const Eigen::Matrix<double, 1, 4> &K);
    void LateralControl();
    void LongitudinalControl();
    std::vector<double> get_command();

  private:
    std::unordered_map<std::string, double> previous;     // 用于临时存储一些上一循环周期的变量
    TrajectoryPoint cur_pose;                                   // 车辆当前状态
    std::vector<double> commands;                         // throttle, steer, brake
    std::vector<Eigen::MatrixXd> lqr_k_table; // LQR离线求解后的k
    ControlPoint desired_point;

    /***********************************横向控制参数**************************************/
    Eigen::MatrixXd Q; // Q矩阵
    Eigen::MatrixXd R; // R矩阵

    /***********************************纵向PID参数**************************************/
    double sum_pid_error; // pid累计误差
    double kp;
    double ki;
    double kd;

  };
} // namespace controller

#endif // CONTROLLER_H
