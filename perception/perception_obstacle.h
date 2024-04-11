#pragma once

#include <Eigen/Eigen>
#include "pnc_point.h"
#include <math.h>
#include <vector>
#include "reference_line.h"

/*障碍物的数据类型*/
class ObstacleInfo {
public:
  TrajectoryPoint center_point;
  int ID; //用于追踪动态障碍物构建虚拟障碍物
  double x_radius;
  double y_radius;//障碍物的形状
  std::vector<TrajectoryPoint> collision_box;
};

class VirtualObs{
public:
  ObstacleInfo obs;
  SLPoint sl_obs;
  ReferenceLine path;
  std::vector<SLPoint> sl_path;
  int desion_flag; //决策变量，1表示左侧超车，2表示右侧超车
};

class PerceptionObstacle {

public:
  PerceptionObstacle() = default;
  ~PerceptionObstacle() = default;
  const std::vector<ObstacleInfo> static_obstacles() const;
  const std::vector<ObstacleInfo> dynamic_obstacles() const;
  const std::vector<VirtualObs> virtual_obstacles() const;
  void set_obstacles(const LocalizationInfo &localization_info, const std::vector<ObstacleInfo> &detected_obstacles);
  const std::vector<ObstacleInfo> get_obstacles() const;
  void CalCollisionBox(ObstacleInfo &obstacle);
  void UpdateVirtualObstacle(std::vector<VirtualObs> virtual_obstacles);
  void FilterAndOutObstacleInfo(const LocalizationInfo &localization_info, std::vector<ObstacleInfo> detected_obstacles); //根据车辆的位置输出障碍物信息

private:
  /* data */
  std::vector<ObstacleInfo> detected_obstacles_;
  std::vector<ObstacleInfo> static_obstacles_;
  std::vector<ObstacleInfo> dynamic_obstacles_;
  std::vector<VirtualObs> virtual_obstacles_;
};
