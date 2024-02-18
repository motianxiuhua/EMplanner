#include "perception/perception_obstacle.h"

const std::vector<ObstacleInfo> PerceptionObstacle::static_obstacles() const {
  return static_obstacles_;
}

const std::vector<ObstacleInfo> PerceptionObstacle::dynamic_obstacles() const {
  return dynamic_obstacles_;
}

const std::vector<ObstacleInfo> PerceptionObstacle::get_obstacles() const {
  return detected_obstacles_;
}

void PerceptionObstacle::set_obstacles(const std::vector<ObstacleInfo> &detected_obstacles){
  detected_obstacles_ = detected_obstacles;
  dynamic_obstacles_.clear();
  static_obstacles_.clear();
  for(ObstacleInfo &obs : detected_obstacles_) {
    CalCollisionBox(obs);
    if(obs.center_point.v > 0.2){
      dynamic_obstacles_.push_back(obs);
    }
    else{
      static_obstacles_.push_back(obs);
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
