/**
 * @file controller.cpp
 * @author czj
 * @brief
 * @version 0.1
 * @date 2023-06-06
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "controller.h"

using namespace std;

namespace controller
{

  /**
   * @brief Construct a new Controller:: Controller object
   *
   * @param lat_control_method 控制方法
   * @param Q Q矩阵
   * @param R R矩阵
   * @param kp
   * @param ki
   * @param kd
   */
  Controller::Controller(
                         const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                         const double &kp, const double &ki, const double &kd
                        )
  {
    //初始化值
    this->commands.resize(3); // throttle, steer, brake 

    this->previous["v"] = 0.0;
    this->previous["t"] = 0.0;
    this->previous["throttle"] = 0.0;
    this->previous["v_error"] = 0.0;
    this->previous["alpha"] = 0.0;
    this->previous["steering"] = 0.0;  

    /***********************************横向控制参数**************************************/
    // Q矩阵
    this->Q = Q;
    // R矩阵
    this->R = R;

    /***********************************纵向PID控制参数**************************************/

    this->sum_pid_error = 0.0; // PID累计误差
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
  }

  double Controller::NormalizeAngle(double &angle)
  {
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  /**
   * @brief 求解离散LQR矩阵
   *
   * @param vx 纵向速度
   * @return Eigen::MatrixXd
   */
  Eigen::MatrixXd Controller::CalcDlqr(double vx, const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R)
  {
    if(vx < 1e-6)
      vx = 1e-6; // 防止速度为0时相除出错
    // A矩阵
    /*
    A matrix (Gear Drive)
    [0.0,                             1.0,                           0.0,                                            0.0;
     0.0,          (cf + cr) / (mass * vx),            -(cf + cr) / mass,              (lf * cf - lr * cr) / (mass * vx);
     0.0,                             0.0,                           0.0,                                            1.0;
     0.0, (lf * cf - lr * cr) / (Iz * vx),     -(lf * cf - lr * cr) / Iz,       (lf * lf * cf + lr * lr * cr) / (Iz * vx);]
    */
    Eigen::Matrix4d A;
    A << 0.0, 1.0, 0.0, 0.0,
        0.0, (cf + cr) / (mass * vx), -(cf + cr) / mass, (lf * cf - lr * cr) / (mass * vx),
        0.0, 0.0, 0.0, 1.0,
        0.0, (lf * cf - lr * cr) / (Iz * vx), -(lf * cf - lr * cr) / Iz, (lf * lf * cf + lr * lr * cr) / (Iz * vx);

    // B矩阵 ：B = [0.0, -cf / mass, 0.0, -lf * cf / Iz]^T

    Eigen::Matrix<double, 4, 1> B;
    B << 0.0, -cf / mass, 0.0, -lf * cf / Iz;

    /***********************************离散化状态方程**************************************/
    double ts = 0.001;
    // 单位矩阵
    Eigen::Matrix4d eye;
    eye.setIdentity(4, 4);
    // 离散化A,B矩阵
    Eigen::Matrix4d Ad;
    Ad = (eye - ts * 0.5 * A).inverse() * (eye + ts * 0.5 * A);
    Eigen::Matrix<double, 4, 1> Bd;
    Bd = B * ts;

    /***********************************求解Riccati方程**************************************/
    int max_iteration = 200; // 设置最大迭代次数
    int tolerance = 0.001;   // 迭代求解精度

    Eigen::Matrix4d P = Q;
    Eigen::Matrix4d P_next;
    for (int i = 0; i < max_iteration; i++)
    {
      P_next = Ad.transpose() * P * Ad -
               Ad.transpose() * P * Bd * (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad + Q;
      if (fabs((P_next - P).maxCoeff()) < tolerance)
      {
        break;
      }
      P = P_next;
    }
    // 求解k值
    Eigen::MatrixXd K;
    K = (R + Bd.transpose() * P * Bd).inverse() * Bd.transpose() * P * Ad;
    return K;
  }

  /**
   * @brief 离线求解LQR表
   *
   * @param vx
   */
  void Controller::CreateLqrOffline(const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R)
  {
    lqr_k_table.clear();
    for (double vx = 0; vx <= 40.0; vx += 0.01)
    {
      lqr_k_table.push_back(CalcDlqr(vx, Q, R));
    }
  }

  /**
   * @brief 控制时间是状态传过来的时间+控制间隔，根据控制时间来计算跟踪的期望值
   * 
   * @param local_waypoints 
   * @param cur_pose 
   */
  void Controller::ComputeControlValue(const std::vector<TrajectoryPoint> &local_waypoints, const TrajectoryPoint &cur_pose)
  {
    double ts = 0.02;//控制时间间隔
    double control_time = cur_pose.t + ts;
    this->cur_pose = cur_pose;
    for(int i = 0; i < local_waypoints.size() - 1; i++)
    {
      if(control_time > local_waypoints[i].t && control_time <= local_waypoints[i+1].t)
      {
        double k = (control_time - local_waypoints[i].t) / 
                   (local_waypoints[i+1].t - local_waypoints[i].t); //斜率
        desired_point.x = local_waypoints[i].x + 
                          k * (local_waypoints[i+1].x - local_waypoints[i].x);
        desired_point.y = local_waypoints[i].y +
                          k * (local_waypoints[i+1].y - local_waypoints[i].y);
        desired_point.heading = local_waypoints[i].heading +
                                k * (local_waypoints[i+1].heading - local_waypoints[i].heading);
        desired_point.v = local_waypoints[i].v +
                          k * (local_waypoints[i+1].v - local_waypoints[i].v); 
        desired_point.kappa = local_waypoints[i].kappa +
                          k * (local_waypoints[i+1].kappa - local_waypoints[i].kappa);  
        desired_point.a = local_waypoints[i].a +
                          k * (local_waypoints[i+1].a - local_waypoints[i].a);                                                                 
      }
    }
  }

  Eigen::Matrix<double, 4, 1> Controller::CalStateError()
  {
    Eigen::Matrix<double, 4, 1> err;

    // 匹配点切向量tor
    Eigen::Matrix<double, 2, 1> tor;
    tor << cos(desired_point.heading), sin(desired_point.heading);

    // 匹配点法向量nor
    Eigen::Matrix<double, 2, 1> nor;
    nor << -sin(desired_point.heading), cos(desired_point.heading);

    // 计算e_d
    Eigen::Matrix<double, 2, 1> d_err;
    d_err << cur_pose.x - desired_point.x, cur_pose.y - desired_point.y;
    double e_d = nor.transpose() * d_err;

    // 计算投影点的角度
    double e_s = tor.transpose() * d_err;
    double projection_theta = desired_point.heading +
                              desired_point.kappa * e_s;

    // 计算e_d_dot
    double e_d_dot = cur_pose.vy * cos(cur_pose.heading - projection_theta) +
                     cur_pose.vx * sin(cur_pose.heading - projection_theta);

    // 计算e_phi,消除角度的多值性(phi+2π与phi相同)
    double e_phi = sin(cur_pose.heading - projection_theta);

    // 计算e_phi_dot
    double s_dot = (cur_pose.vx * cos(cur_pose.heading - projection_theta) -
                    cur_pose.vy * sin(cur_pose.heading - projection_theta)) /
                   (1 - desired_point.kappa * e_d);
    double phi_dot = cur_pose.vx * desired_point.heading;
    double e_phi_dot = phi_dot - s_dot * desired_point.kappa;

    err << e_d,
        e_d_dot,
        e_phi,
        e_phi_dot;
    return err;
  }

  double Controller::CalForwardAngle(const Eigen::Matrix<double, 1, 4> &K)
  {
    double k3 = K[2];
    // 不足转向系数
    double kv = lr * mass / (cf * L) - lf * mass / (cr * L);

    double forward_angle = L * desired_point.kappa - kv * cur_pose.vx * cur_pose.vx * desired_point.kappa -
                           k3 * (lr * desired_point.kappa + lf * mass * cur_pose.vx * cur_pose.vx * desired_point.kappa / (lr * cr));

    // double forward_angle = desired_point.kappa * (lf + lr - lr * k3 -
    //                               (mass * pow(cur_pose.vx, 2) / (lf + lr)) * (lr / cf + lf * k3 / cr - lf / cr));
    return forward_angle;
  }

  void Controller::LateralControl()
  {
    double steering;
    // 计算横向误差
    Eigen::Matrix<double, 4, 1> err = CalStateError();

    // 查表获取LQR k值
    if (cur_pose.vx < 1e-6)
    {
      steering = 0;
    }
    else
    {
      int index = static_cast<int>(cur_pose.vx / 0.01); //除以的系数取决于LQR K表制作时vx的递增值 
      Eigen::Matrix<double, 1, 4> K_correspond_vx = lqr_k_table[index];
      // 计算前馈转角
      double forward_angle = CalForwardAngle(K_correspond_vx);
      NormalizeAngle(forward_angle);
      steering = forward_angle - K_correspond_vx * err;
    }
    NormalizeAngle(steering);
    if (steering >= max_degree)
      steering = max_degree;
    else if (steering <= -max_degree)
      steering = -max_degree;
    commands[1] = steering;
  }

  void Controller::LongitudinalControl()
  {
    double ts = 0.02;//控制时间间隔
    double v_error = desired_point.v - cur_pose.v;
    sum_pid_error += v_error * ts;
    double k_ = this->kp * v_error;
    double integral_ = this->ki * sum_pid_error;
    double diff_ = this->kd * (v_error - previous["v_error"]) / ts;

    double throttle = k_ + integral_ + diff_;

    previous["v_error"] = v_error;
    previous["throttle"] = throttle;
    previous["v_error"] = v_error;
    if(throttle < 0)
    {
      commands[0] = 0;
      commands[2] = -throttle;
    }
    else
    {
      commands[0] = throttle;
      commands[2] = 0;
    }    
  }

  std::vector<double> Controller::get_command()
  {
    std::vector<double> cmd(3);
    if(desired_point.v < 0.01)
    {
      cmd[0] = 0;
      cmd[1] = 0;
      cmd[2] = 1;
    }
    //转化为Carla的值
    cmd[0] = max(min(commands[0], 1.0), 0.0);
    cmd[1] = commands[1] / max_degree;
    cmd[2] = max(min(commands[2], 1.0), 0.0);
    return cmd;
  }
} // controller
