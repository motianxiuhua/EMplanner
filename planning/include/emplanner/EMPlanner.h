/*
此类的功能：
通过动态规划和二次规划方法输出轨迹
*/
#pragma once

#include "perception_obstacle.h"
#include "reference_line.h"
#include "trajectory.h"
#include <memory>

#include "emplanner/path_time_graph.h"
#include "emplanner/speed_time_graph.h"

class EMPlanner {
public:
  EMPlanner(const std::unordered_map<std::string, double> &conf);
  ~EMPlanner() = default;

 std::pair<std::unique_ptr<PathTimeGraph>,std::unique_ptr<SpeedTimeGraph>>
       Plan(const TrajectoryPoint &planning_init_point, //规划起点
            const ReferenceLine &reference_line,        //参考线
            const LocalizationInfo &localization,       //定位信息
            const std::vector<VirtualObs> &pre_virtual_obstacles,
            const std::vector<ObstacleInfo> &static_obstacles,  //障碍物信息
            const std::vector<ObstacleInfo> &dynamic_obstacles,
            Trajectory *trajectory,                          //输出轨迹
            std::vector<VirtualObs> &virtual_obstacles); //虚拟障碍物

  // 1.计算规划起点
  void CalPlaningStartPoint(const Trajectory &pre_traj,
                            const LocalizationInfo &local_info,
                            TrajectoryPoint *start_plan_point,
                            Trajectory *stitch_traj
                            );

  void StitchTrajectory(const Trajectory &cur_traj,
                        const Trajectory &stitch_traj, Trajectory &final_traj);
  // 2.计算起点的sl

  // 2.静态障碍物的投影SL

  // 3.采样点

  // 4.动态规划

  // 5.二次规划

  //直角坐标系转化为直角坐标系。
  /*
   *1.根据自车位置和参考线建立sl的s轴
   *2.xy->找到该点在参考线的投影点，即求出了s
   *3.l=(x-r)*N x自车矢量，r投影点矢量，N投影点法向量
   */
  // 1.dp_pt_graph包含参考线、自车位置sl、障碍物位置sl；采样行和列，

  // const std::unique_ptr<PathTimeGraph> sl_graph() const;
  // const std::unique_ptr<SpeedTimeGraph> st_graph() const;

  std::unique_ptr<PathTimeGraph> sl_graph_;
  std::unique_ptr<SpeedTimeGraph> st_graph_;

private:
  std::unordered_map<std::string, double> config_;

  Trajectory pre_trajectory_;
  Trajectory trajectory_;

  // 1.参考线
  // 2.车辆的sl
  // 3.障碍物的sl
  // 4.采样点的sl
};
