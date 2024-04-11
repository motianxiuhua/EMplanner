#include "perception_obstacle.h"

const std::vector<ObstacleInfo> PerceptionObstacle::static_obstacles() const {
  return static_obstacles_;
}

const std::vector<ObstacleInfo> PerceptionObstacle::dynamic_obstacles() const {
  return dynamic_obstacles_;
}

const std::vector<ObstacleInfo> PerceptionObstacle::get_obstacles() const {
  return detected_obstacles_;
}

const std::vector<VirtualObs> PerceptionObstacle::virtual_obstacles() const {
  return virtual_obstacles_;
}

void PerceptionObstacle::FilterAndOutObstacleInfo(const LocalizationInfo &localization_info, std::vector<ObstacleInfo> detected_obstacles){
  std::vector<ObstacleInfo> obstacles = detected_obstacles;
  detected_obstacles.clear();
  // 切向量
  auto tor = std::make_pair(cos(localization_info.heading), sin(localization_info.heading));
  //法向量
  auto nor =
      std::make_pair(-sin(localization_info.heading), cos(localization_info.heading));
  for(int i = 0; i<obstacles.size(); i++)
  {
    //% 误差向量
    auto d_err = std::make_pair(localization_info.x - obstacles[i].center_point.x,
                                localization_info.y - obstacles[i].center_point.y);
    //%纵向误差
    double lon_err = abs(d_err.first * tor.first + d_err.second * tor.second);
    //%横向误差
    double lat_err = abs(d_err.first * nor.first + d_err.second * nor.second);
    if(lon_err<50 && lon_err>-10 &&lat_err>-10 && lat_err<10){
      detected_obstacles.push_back(obstacles[i]);
    }
  }
}

void PerceptionObstacle::set_obstacles(const LocalizationInfo &localization_info, const std::vector<ObstacleInfo> &detected_obstacles){
  detected_obstacles_ = detected_obstacles;
  dynamic_obstacles_.clear();
  static_obstacles_.clear();
  FilterAndOutObstacleInfo(localization_info, detected_obstacles_);
  for(ObstacleInfo &obs : detected_obstacles_) {
    CalCollisionBox(obs);
    if(obs.center_point.v > 0.01){
      dynamic_obstacles_.push_back(obs);
    }
    else{
      static_obstacles_.push_back(obs);
    }
  }
  if (!virtual_obstacles_.empty())
  {
    for(auto virtual_obs:virtual_obstacles_) 
    {
      static_obstacles_.push_back(virtual_obs.obs);
    }
    
  }
}

void PerceptionObstacle::CalCollisionBox(ObstacleInfo &obstacle){
  std::vector<TrajectoryPoint> collision_box(8);
  double x = obstacle.center_point.x;
  double y = obstacle.center_point.y;
  double heading = obstacle.center_point.heading;
  double x_rad = obstacle.x_radius;
  double y_rad = obstacle.y_radius;

  // 获取BOX边上8个点的坐标矩阵
  Eigen::MatrixXd position_matrix(8, 2), translation_matrix(8, 2), rotation_matrix(2, 2);

  // 旋转矩阵
  position_matrix << x, y,
      x, y,
      x, y,
      x, y,
      x, y,
      x, y,
      x, y,
      x, y;

  translation_matrix << -x_rad, -y_rad,
      -x_rad, 0,
      -x_rad, y_rad,
      0, y_rad,
      x_rad, y_rad,
      x_rad, 0,
      x_rad, -y_rad,
      0, -y_rad;

  rotation_matrix << cos(heading), sin(heading),
      -sin(heading), cos(heading);

  position_matrix = translation_matrix * rotation_matrix + position_matrix;

  for (int i = 0; i < position_matrix.rows(); i++)
  {
      collision_box[i].x = position_matrix(i, 0);
      collision_box[i].y = position_matrix(i, 1);
      collision_box[i].heading = obstacle.center_point.heading;
      collision_box[i].vx = obstacle.center_point.vx;
      collision_box[i].vy = obstacle.center_point.vy;
      collision_box[i].v = obstacle.center_point.v;
  }
  obstacle.collision_box = collision_box;
}

 void PerceptionObstacle::UpdateVirtualObstacle(std::vector<VirtualObs> virtual_obstacles){
  virtual_obstacles_ = virtual_obstacles;
  for(int i = 0; i < virtual_obstacles_.size(); i++)
  {
    CalCollisionBox(virtual_obstacles_[i].obs);
  }
 }
