#pragma once

#include <Eigen/Eigen>
#include "pnc_point.h"
#include <math.h>
#include <vector>

/*障碍物的数据类型*/
class ObstacleInfo {
public:
  TrajectoryPoint center_point;
  int ID; //用于追踪动态障碍物构建虚拟障碍物
  double x_radius;
  double y_radius;//障碍物的形状
  std::vector<TrajectoryPoint> collision_box;
};

class PerceptionObstacle {

public:
  PerceptionObstacle() = default;
  ~PerceptionObstacle() = default;
  const std::vector<ObstacleInfo> static_obstacles() const;
  const std::vector<ObstacleInfo> dynamic_obstacles() const;
  void set_obstacles(const std::vector<ObstacleInfo> &detected_obstacles);
  const std::vector<ObstacleInfo> get_obstacles() const;
  void CalCollisionBox(ObstacleInfo &obstacle);
  void UpdateVirtualObstacle(std::vector<ReferencePoint> xy_virtual_obstacles);
  void FilterAndOutObstacleInfo(
      LocalizationInfo localization_info); //根据车辆的位置输出障碍物信息

private:
  /* data */
  std::vector<ObstacleInfo> detected_obstacles_;
  std::vector<ObstacleInfo> static_obstacles_;
  std::vector<ObstacleInfo> dynamic_obstacles_;
};
