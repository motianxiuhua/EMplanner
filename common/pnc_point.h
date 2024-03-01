/**
 * @file pnc_point.h
 * @author xiuhua_liang (xiuhua_liang@163.com)
 * @brief 规划与控制的点定义
 * @version 0.1
 * @date 2024-01-15
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#pragma once

class MapPoint {
public:
  /* data */
  double x;
  double y;
  double heading; //方向角
};

class ReferencePoint : public MapPoint {
public:
  /* data */
  double index;
  double kappa;   //曲率
  double dkappa;  //曲率的导数
};

class TrajectoryPoint : public ReferencePoint {
  /* data */
public:
  double v;  //速度
  double vx; //车身坐标系
  double vy;
  double yaw_rate;
  double a; //加速度
  double ax;
  double ay;
  double t; //时间
};

class ControlPoint: public MapPoint{
public:
  double kappa;
  double v;
  double a;
};

class LocalizationInfo : public TrajectoryPoint {
public:
    bool is_odometry;
};

// SL图点坐标
class SLPoint {
private:
  /* data */
public:
  int index; //对应参考线的索引
  double s;
  double ds_dt;
  double dds_dt;
  double l;
  double dl_dt;
  double ddl_dt;
  double dddl_dt;
  double dl_ds;
  double ddl_ds;
  double dddl_ds;

  double cost2start; //起点到该点的cost  这些是对于采样点来说的，其他的用不上
  int pre_mincost_row; //最小cost前一列的行号
};

class STPoint {
public:
  double t;
  double s;
  double ds_dt;
  double dds_dt;

  double cost2start;
  int pre_mincost_row;
};

class STLine {
public:
  STPoint in_point;//自车和障碍物车纵向距离切入时的参数
  STPoint out_point;//自车和障碍物车纵向距离切出时的参数
};