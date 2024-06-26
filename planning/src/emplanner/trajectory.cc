#include "emplanner/trajectory.h"

const std::vector<TrajectoryPoint> Trajectory::trajectory_points() const {
  return trajectory_points_;
}

void Trajectory::set_trajectory_points(
    const std::vector<TrajectoryPoint> &trajectory_points) {
  trajectory_points_ = trajectory_points;
}

void Trajectory::clear_trajectory_points(){
  trajectory_points_.clear();
}

Trajectory::Trajectory(/* args */) {}

Trajectory::~Trajectory() {}
