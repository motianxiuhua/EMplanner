#include "emplanner/speed_time_graph.h"

/// @brief
/// @param emplaner_conf
SpeedTimeGraph::SpeedTimeGraph(const ReferenceLine &planning_path, const std::vector<ObstacleInfo> &dynamic_obstacles,
                               const std::vector<VirtualObs> &pre_virtual_obstacles,
                               const std::unordered_map<std::string, double> &emplaner_conf) //传入规划好的路径
{
  planning_path_ = planning_path;
  dynamic_obstacles_ = dynamic_obstacles;
  emplaner_conf_ = emplaner_conf;
  pre_virtual_obstacles_ = pre_virtual_obstacles;
  InitSAxis(planning_path_);
}
// 1.基于规划的轨迹，初始化坐标轴
void SpeedTimeGraph::InitSAxis(const ReferenceLine planning_path) {
  std::vector<ReferencePoint> planning_path_points =
      planning_path.reference_points();
  int size = planning_path_points.size();
  std::vector<SLPoint> sl_planning_path;
  sl_planning_path.resize(size);
  sl_planning_path[0].s = 0;
  sl_planning_path[0].index = 0;

  double ds2start = 0;
  for (int i = 1; i < size; i++) {
    ds2start =
        ds2start +
        sqrt(pow(planning_path_points[i].x - planning_path_points[i - 1].x, 2) +
             pow(planning_path_points[i].y - planning_path_points[i - 1].y, 2));
    sl_planning_path[i].s = ds2start;
    sl_planning_path[i].index = i;
  }
  sl_planning_path_ = sl_planning_path;
}

void SpeedTimeGraph::SetStartState(const TrajectoryPoint &plan_start_point) {

  st_plan_start_.s = 0; // 采用的是相对坐标系
  st_plan_start_.t = 0;
  Eigen::Vector2d tor(cos(plan_start_point.heading),sin(plan_start_point.heading));
  Eigen::Vector2d velocity(plan_start_point.vx, plan_start_point.vy);
  Eigen::Vector2d accl(plan_start_point.ax, plan_start_point.ay);
  st_plan_start_.ds_dt = tor.transpose() * velocity; // 相对路径规划的sl参考系
  st_plan_start_.dds_dt = tor.transpose() * accl;
}

// 2.计算障碍物的ST位置
void SpeedTimeGraph::SetDynamicObstaclesSL() {
  std::vector<MapPoint> obstacles_xy(dynamic_obstacles_.size());

  for (int i = 0; i < dynamic_obstacles_.size(); i++) {
    obstacles_xy[i].x = dynamic_obstacles_[i].center_point.x;
    obstacles_xy[i].y = dynamic_obstacles_[i].center_point.y;
  }
  std::vector<ReferencePoint> match_points;
  std::vector<ReferencePoint> project_points;
  
  //此部分比较妥协，暂时使用这种赋值循环，解决障碍物信息和参数不一致得到的问题
  std::vector<TrajectoryPoint> traj_points(dynamic_obstacles_.size());
  for (int i = 0; i < dynamic_obstacles_.size(); i++) {
    traj_points[i].x = dynamic_obstacles_[i].center_point.x;
    traj_points[i].y = dynamic_obstacles_[i].center_point.y;
    traj_points[i].vx = dynamic_obstacles_[i].center_point.vx;
    traj_points[i].vy = dynamic_obstacles_[i].center_point.vy;
    traj_points[i].ax = dynamic_obstacles_[i].center_point.ax;
    traj_points[i].ay = dynamic_obstacles_[i].center_point.ay;
  }
  
  for (int i = 0; i < dynamic_obstacles_.size(); i++) {
    bool vir_flag = false;
    std::vector<SLPoint> sl_dynamic_obstacles; 
    std::vector<TrajectoryPoint> traj_point;
    std::vector<MapPoint> obs_xy;
    traj_point.push_back(traj_points[i]);
    obs_xy.push_back(obstacles_xy[i]);
    for(auto vir_obs : pre_virtual_obstacles_)
    {
      if (vir_obs.obs.ID == dynamic_obstacles_[i].ID){
        ReferenceLineProvider::FindMatchAndProjectPoint(
        vir_obs.path, obs_xy, 0, 30, match_points, project_points);
        PathTimeGraph::Cartesian2Frenet(vir_obs.path, vir_obs.sl_path,
                              traj_point, match_points, project_points,
                              sl_dynamic_obstacles);
        vir_flag=true;
        break;
      }
    }
    if(!vir_flag) 
    {
        ReferenceLineProvider::FindMatchAndProjectPoint(
        planning_path_, obs_xy, 0, 30, match_points, project_points);
        PathTimeGraph::Cartesian2Frenet(planning_path_, sl_planning_path_,
                                traj_point, match_points, project_points,
                                sl_dynamic_obstacles);
    }
    sl_dynamic_obstacles_.push_back(sl_dynamic_obstacles.front());
  }
}

void SpeedTimeGraph::GenerateSTGraph(TrajectoryPoint host_vehicle) {
  if (st_plan_start_.ds_dt < 2)
    return;
  //该函数生成st图，使用斜直线模型
  for (int i = 0; i < sl_dynamic_obstacles_.size(); ++i) {
    // 1.横向/侧向移动缓慢的障碍物,可能是跟车场景和对向行驶场景
    if (abs(sl_dynamic_obstacles_[i].dl_dt) < 0.3) {

      if (abs(sl_dynamic_obstacles_[i].l) > 2 || sl_dynamic_obstacles_[i].ds_dt >= st_plan_start_.ds_dt)
        continue; //横向距离太远，速度规划直接忽略;或者如果障碍物速度大于自车速度则忽略
      else { 
        //侧向移动缓慢，并且速度比自车速度小
        /*
        % TODO需要做虚拟障碍物
        % 这里由于算力原因没做逻辑
        %
        如何做：感知模块加逻辑，给出障碍物跟踪，判断两帧之间的感知所看到的障碍物是否为同一个
        %         速度规划模块在一开始先给出虚拟障碍物决策，不做处理
        % 下一帧，路径规划拿到虚拟障碍物标记，规划出绕过去的路径/跟车路径 %
        速度规划计算出速度，绕过去/跟车 % 本算法欠缺的：障碍物结构体
        结构体不仅要包含坐标 速度 还要包含
        决策标记(是否为虚拟障碍物，左绕还是右绕，避让还是超车) %
        感知模块，判断两帧之间的障碍物是否为同一个 %              算力
        */
        /*1.计算在SL碰撞位置
         2.自然坐标系转换为世界坐标
        */
       //TODO carla是有障碍物跟踪的，根据ID来判断当前障碍物和上一次障碍物是否为同一个
       
       // 如果没有标记，则需要判断是否需要标记虚拟障碍物
       // 如果自车的速度比前车的速度大很多，则需要超车
       if(sl_dynamic_obstacles_[i].ds_dt < st_plan_start_.ds_dt * 0.8){
          bool pre_flag = false;
          VirtualObs pre_virtual_obstacle;
          for(auto vir_obs : pre_virtual_obstacles_)
          {
            if (vir_obs.obs.ID == dynamic_obstacles_[i].ID){
              pre_flag = true;//虚拟障碍物标记过
              pre_virtual_obstacle = vir_obs;
            }
          }
          VirtualObs virtual_obstacle;
          std::vector<SLPoint> sl_virtual_obstacles;
          std::vector<ReferencePoint> xy_virtual_obstacles;
          virtual_obstacle.obs = dynamic_obstacles_[i];
          if (pre_virtual_obstacles_.empty() || !pre_flag){                    
              double t_collision = (sl_dynamic_obstacles_[i].s) /
                                  (st_plan_start_.ds_dt - sl_dynamic_obstacles_[i].ds_dt); //碰撞发生的时间       
              double s_collision =
                  sl_dynamic_obstacles_[i].s + (t_collision ) * sl_dynamic_obstacles_[i].ds_dt; //碰撞发生的位置
              if (s_collision > sl_planning_path_.back().s){
                continue;
              } 
              double l_collision = sl_dynamic_obstacles_[i].l;
              virtual_obstacle.sl_obs.s = s_collision;
              virtual_obstacle.sl_obs.l = sl_dynamic_obstacles_[i].l; // 这里没有做纵向推演，但实际上进入该if语句的纵向速度就是很小，所以不做也行
              sl_virtual_obstacles.push_back(virtual_obstacle.sl_obs);
              
              virtual_obstacle.path = planning_path_;
              virtual_obstacle.sl_path = sl_planning_path_;
          } 
          else{
            std::vector<MapPoint> map_points;
            map_points.resize(1);
            map_points[0].x = host_vehicle.x;
            map_points[0].y = host_vehicle.y;
            //定义变量，用于存储计算结果
            std::vector<ReferencePoint> host_match_points, host_project_points;
            ReferenceLineProvider::FindMatchAndProjectPoint(pre_virtual_obstacle.path, map_points,
                                                            0, 3, host_match_points,
                                                            host_project_points);
            std::vector<TrajectoryPoint> host_point_vec;
            host_point_vec.push_back(host_vehicle);
            std::vector<SLPoint> host_sl;
            PathTimeGraph::Cartesian2Frenet(pre_virtual_obstacle.path, pre_virtual_obstacle.sl_path, host_point_vec,
                            host_match_points, host_project_points, host_sl);
            double t_collision = (sl_dynamic_obstacles_[i].s - host_sl[0].s) /
                                  (host_sl[0].ds_dt - sl_dynamic_obstacles_[i].ds_dt); //碰撞发生的时间
            double s_collision =
                sl_dynamic_obstacles_[i].s +  (t_collision )  * sl_dynamic_obstacles_[i].ds_dt; //碰撞发生的位置
            if (sl_dynamic_obstacles_[i].s + 5 <= host_sl[0].s  || s_collision > pre_virtual_obstacle.sl_path.back().s){
              continue;
            }
            double l_collision = sl_dynamic_obstacles_[i].l;
            virtual_obstacle.sl_obs.s = s_collision;
            virtual_obstacle.sl_obs.l = sl_dynamic_obstacles_[i].l; // 这里没有做纵向推演，但实际上进入该if语句的纵向速度就是很小，所以不做也行
            sl_virtual_obstacles.push_back(virtual_obstacle.sl_obs);

            virtual_obstacle.path = pre_virtual_obstacle.path;
            virtual_obstacle.sl_path = pre_virtual_obstacle.sl_path;
          }
          PathTimeGraph::Frenet2Cartesian(virtual_obstacle.path, virtual_obstacle.sl_path,
                                              sl_virtual_obstacles,
                                              xy_virtual_obstacles);
          virtual_obstacle.obs.center_point.x = xy_virtual_obstacles[0].x;
          virtual_obstacle.obs.center_point.y = xy_virtual_obstacles[0].y;
          virtual_obstacle.obs.center_point.heading= xy_virtual_obstacles[0].heading;
          virtual_obstacle.obs.center_point.kappa = xy_virtual_obstacles[0].kappa;
          virtual_obstacles_.push_back(virtual_obstacle);
       }
       //TODO否则，跟车

        // 如果之前标记了虚拟障碍物，则需要在标记的虚拟障碍物对应的参考轨迹上投影来计算新的虚拟障碍物位置
        continue;
      }
    }
    //有比较大的横向速度
    /*TODO这里动态障碍没有考虑大小和形状，第一障碍物在ST图的形状不一定是长方形
    如果纵向也有移动速度，那么是个平行四边形，因为相同的纵向距离s不在同一个时间t
    */
    //计算切入切出时间
    // t_zero 为动态障碍物的l到0，所需要的时间
    double t_zero = -sl_dynamic_obstacles_[i].l / sl_dynamic_obstacles_[i].dl_dt; //时间等于路程除以速度
    //计算进出±2的时间
    double t_boundary1 = 2.0 / sl_dynamic_obstacles_[i].dl_dt + t_zero;
    double t_boundary2 = -2.0 / sl_dynamic_obstacles_[i].dl_dt + t_zero;
    double t_max = 0;
    double t_min = 0;
    if (t_boundary1 > t_boundary2) {
      t_max = t_boundary1;
      t_min = t_boundary2;
    }

    else {
      t_max = t_boundary2;
      t_min = t_boundary1;
    }
    //鬼探头和远方车辆
    if (t_max < 1 || t_min > 8)
      continue;
    /*
    % 对于切入切出太远的，或者碰瓷的，忽略
    %
    车辆运动是要受到车辆动力学制约的，如果有碰瓷的，即使规划出了很大的加速度，车辆也执行不了
    % 碰瓷障碍物也需要做虚拟障碍物和路径规划一起解决。速度规划无法解决
    */
    // % 在感知看到的时候，障碍物已经在+-2的内部了
    STLine st_obstacle;
    if (t_min < 0 && t_max > 0) {
      st_obstacle.in_point.s = sl_dynamic_obstacles_[i].s;
      st_obstacle.in_point.t = 0;
      st_obstacle.out_point.s = sl_dynamic_obstacles_[i].s + sl_dynamic_obstacles_[i].ds_dt * t_max;
      st_obstacle.out_point.t = t_max;
    } else //正常障碍物
    {
      st_obstacle.in_point.s = sl_dynamic_obstacles_[i].s + sl_dynamic_obstacles_[i].ds_dt * t_min;
      st_obstacle.in_point.t = t_min;
      st_obstacle.out_point.s = sl_dynamic_obstacles_[i].s + sl_dynamic_obstacles_[i].ds_dt * t_max;
      st_obstacle.out_point.t = t_max;
    }
    st_obstacles_.push_back(st_obstacle);
  }
}

// 3.采样
void SpeedTimeGraph::CreateSmaplePoint() { // row=40,col=16;

  /*% 时间从0到8开始规划，最多8秒
  % s的范围从0开始到路径规划的path的总长度为止
  % 为了减少算力 采用非均匀采样，s越小的越密，越大的越稀疏
  */
  int row = emplaner_conf_["speed_dp_sample_row"];
  int col = emplaner_conf_["speed_dp_sample_col"];
  double dt = emplaner_conf_["speed_dp_sample_t"];
  double ds = emplaner_conf_["speed_dp_sample_s"];
  sample_points_.resize(row);

  for (int i = 0; i < row; i++) { //  
    sample_points_[i].resize(col);
    for (int j = 0; j < col; j++) {
      sample_points_[i][j].s = i * ds;
      sample_points_[i][j].t = (j + 1) * dt;
      sample_points_[i][j].cost2start = DBL_MAX;
    }
  }
}

// 4.动态规划
void SpeedTimeGraph::SpeedDynamicPlanning() {
  int row = emplaner_conf_["speed_dp_sample_row"];
  int col = emplaner_conf_["speed_dp_sample_col"];

  for (int i = 0; i < row; i++) {
    // 第一列的前一个节点只有起点，起点的s t 都是0
    sample_points_[i][0].ds_dt =
        sample_points_[i][0].s / sample_points_[i][0].t;
    sample_points_[i][0].dds_dt =
        (sample_points_[i][0].ds_dt - st_plan_start_.ds_dt) /
        sample_points_[i][0].t; //TODO速度用于cost计算，但加速度并没有什么用

    sample_points_[i][0].cost2start =
        CalcDpCost(st_plan_start_, sample_points_[i][0]);
    sample_points_[i][0].pre_mincost_row = -1;
  }

  //动态规划主程序
  for (int j = 1; j < col; j++) {
    for (int i = 0; i < row; i++) {
      for (int k = 0; k <= i; k++) {
        double cost =
            sample_points_[k][j - 1].cost2start +
            CalcDpCost(sample_points_[k][j - 1], sample_points_[i][j]);
        if (cost < sample_points_[i][j].cost2start) {
          sample_points_[i][j].cost2start = cost;
          //保留最优的点的ds_dt
          sample_points_[i][j].ds_dt =
              (sample_points_[i][j].s - sample_points_[k][j - 1].s) /
              (sample_points_[i][j].t - sample_points_[k][j - 1].t);
          sample_points_[i][j].pre_mincost_row = k;
        }
      }
    }
  }

  //% 找到dp_node_cost 上边界和右边界代价最小的节点
  double min_cost = DBL_MAX;
  int min_row;
  int min_col;
  for (int i = 0; i < row; i++) {
    if (sample_points_[i][col - 1].cost2start < min_cost) {
      min_cost = sample_points_[i][col - 1].cost2start;
      min_row = i;
      min_col = col - 1;
    }
  }

  for (int j = 0; j < col; j++) {
    if (sample_points_[row - 1][j].cost2start < min_cost) {
      min_cost = sample_points_[row - 1][j].cost2start;
      min_row = row - 1;
      min_col = j;
    }
  }
  std::vector<STPoint> dp_speed_points(min_col + 1);

  dp_speed_points[min_col] = sample_points_[min_row][min_col];
  for (int j = min_col; j >= 1; j--) {
    int pre_mincost_row = dp_speed_points[j].pre_mincost_row;
    dp_speed_points[j - 1] = sample_points_[pre_mincost_row][j - 1];
  }

  dp_speed_points.insert(dp_speed_points.begin(), st_plan_start_);
  //插入规划起点
  dp_speed_points_ = dp_speed_points;
}
double SpeedTimeGraph::CalcDpCost(STPoint &point_s, STPoint &point_e) {
  /*
          % 该函数将计算链接两个节点之间边的代价
            % 输入：边的起点的行列号row_start,col_start
          边的终点行列号row_end,col_end %
          障碍物st信息obs_st_s_in_set,obs_st_s_out_set,obs_st_t_in_set,obs_st_t_out_set
                  %
          推荐速度代价权重w_cost_ref_speed,加速度代价权重w_cost_accel,障碍物代价权重w_cost_obs
                  % 推荐速度reference_speed
                  % 拼接起点的速度plan_start_s_dot
                  % s_list,t_list 采样距离
                  % dp_st_s_dot 用于计算加速度
            % 首先计算终点的st坐标
        */

  double cur_s_dot =
      (point_e.s - point_s.s) / (point_e.t - point_s.t); //计算速度
  double cur_s_dot2 =
      (cur_s_dot - point_s.ds_dt) / (point_e.t - point_s.t); //计算加速度
  //计算推荐速度代价
  double cost_ref_speed = emplaner_conf_["speed_dp_cost_ref_speed"] *
                          pow(cur_s_dot - emplaner_conf_["ref_speed"], 2);
  //  % 计算加速度代价，这里注意，加速度不能超过车辆动力学上下限
  double cost_accel = 0;
  if (cur_s_dot2 < 5 && cur_s_dot2 > -6)
    cost_accel = emplaner_conf_["speed_dp_cost_accel"] * pow(cur_s_dot2, 2);
  else
    // % 超过车辆动力学限制，代价会增大很多倍
    cost_accel =
        100000.0 * emplaner_conf_["speed_dp_cost_accel"] * pow(cur_s_dot2, 2);

  double cost_obs = CalcObsCost(point_s, point_e);

  return cost_ref_speed + cost_accel + cost_obs;
}

double SpeedTimeGraph::CalcObsCost(const STPoint &point_s,
                                   const STPoint &point_e) {
  /*
  % 该函数将计算边的障碍物代价
  % 输入：边的起点终点s_start,t_start,s_end,t_end
  %       障碍物信息
  obs_st_s_in_set,obs_st_s_out_set,obs_st_t_in_set,obs_st_t_out_set %
  障碍物代价权重w_cost_obs % 输出：边的障碍物代价obs_cost % 输出初始化
  */
  double cost_obs = 0;
  int n = 5;
  double dt = (point_e.t - point_s.t) / (n - 1);
  double k = (point_e.s - point_s.s) / (point_e.t - point_s.t);
  for (int i = 0; i < n; i++) {
    double t = point_s.t + i * dt;
    double s = point_s.s + k * i * dt;
    double min_dis = 0;
    for (const auto &obs :
         st_obstacles_) { //计算路径点到障碍物的距离，点到线的距离。
      //计算点到线的距离，如果垂线在三角形内
      Eigen::Vector2d vector1(obs.in_point.t - t, obs.in_point.s - s);
      Eigen::Vector2d vector2(obs.out_point.t - t, obs.out_point.s - s);
      Eigen::Vector2d vector3 = vector2 - vector1;
      double dis1 = sqrt(vector1.transpose() * vector1);
      double dis2 = sqrt(vector2.transpose() * vector2);
      double dis3 = abs(vector1.transpose() * vector2) /
                    sqrt(vector3.transpose() * vector3);
      if ((vector1.transpose() * vector3 > 0 &&
           vector2.transpose() * vector3 > 0) ||
          (vector1.transpose() * vector3 < 0 &&
           vector2.transpose() * vector3 < 0))
        min_dis = std::min(dis1, dis2);
      else
        min_dis = dis3;
    }
    cost_obs =
        cost_obs + CalcCollisionCost(emplaner_conf_["speed_dp_cost_obs"], min_dis);
  }
  return cost_obs;
}

double SpeedTimeGraph::CalcCollisionCost(double w_cost_obs, double min_dis) {
  double collision_cost = 0;
  if (abs(min_dis) < 0.5)
    collision_cost = 10e6;
  else if (abs(min_dis) >= 0.5 && abs(min_dis) < 5)
    //  % min_dis = 0.5 collision_cost = w_cost_obs ^ 1;
    // % min_dis = 1.5 collision_cost = w_cost_obs ^ 0 = 1
    collision_cost = w_cost_obs * pow(1000, 2 - min_dis);
  else
    collision_cost = 0;

  return collision_cost;
}

void SpeedTimeGraph::GenerateCovexSpace() {
  int n = dp_speed_points_.size();
  convex_s_lb_ = Eigen::VectorXd::Zero(n);
  convex_s_ub_ = Eigen::VectorXd::Ones(n) * DBL_MAX;
  convex_ds_dt_lb_ = Eigen::VectorXd::Ones(n) * (-DBL_MAX);
  convex_ds_dt_ub_ = Eigen::VectorXd::Ones(n) * DBL_MAX;

  int path_end_index = sl_planning_path_.size();
  int dp_end_index = dp_speed_points_.size();

  //此处不用找缓存
  //此处施加动力学约束，通过曲率和横向加速度计算速度限制
  for (int i = 0; i < n; i++) {
    double cur_s = dp_speed_points_[i].s;
    //搜索，可以用二分搜索
    //计算当前点的曲率，通过插值法
    int cur_index = 0;
    for (cur_index = 0; cur_index < path_end_index - 1; cur_index++) {
      if (cur_s >= sl_planning_path_[cur_index].s &&
          cur_s < sl_planning_path_[cur_index + 1].s)
        break;
    }
    planning_path_points_ = planning_path_.reference_points();
    double k =
        (planning_path_points_[cur_index + 1].kappa -
         planning_path_points_[cur_index].kappa) /
        (sl_planning_path_[cur_index + 1].s - sl_planning_path_[cur_index].s);
    double cur_kappa = planning_path_points_[cur_index].kappa +
                       k * (cur_s - sl_planning_path_[cur_index].s);

    double max_speed =
        sqrt(emplaner_conf_["max_lateral_accel"] / (abs(cur_kappa) + 1e-10));
    double min_speed = 0;
    convex_ds_dt_lb_(i) = min_speed;
    convex_ds_dt_ub_(i) = max_speed;
  }

  for (const auto &obs : st_obstacles_) {
    //  % 取s t 直线的中点，作为obs_s obs_t 的坐标
    double obs_t = (obs.in_point.t + obs.out_point.t) / 2;
    double obs_s = (obs.in_point.s + obs.out_point.s) / 2;
    // 计算障碍物的纵向速度
    double obs_speed = (obs.out_point.s - obs.in_point.s) /
                       (obs.out_point.t - obs.in_point.t);
    // % 插值找到当t = obs_t时，动态规划的s 的值，用于判断让行还是超车
    double mid_index = FindDpMatchIndex(obs_t);
    double mid_k =
        (dp_speed_points_[mid_index + 1].s - dp_speed_points_[mid_index].s) /
        (dp_speed_points_[mid_index + 1].t - dp_speed_points_[mid_index].t);
    double dp_s = dp_speed_points_[mid_index].s +
                  mid_k * (obs_t - dp_speed_points_[mid_index].t);

    // t_in,t_out对应的动态规划点
    int t_lb_index = FindDpMatchIndex(obs.in_point.t);

    int t_ub_index = FindDpMatchIndex(obs.out_point.t);

    // % 这里稍微做个缓冲，把 t_lb_index 稍微缩小一些，t_ub_index稍微放大一些
    t_lb_index = std::max(t_lb_index - 2, 1);
    //% 最低为3 因为碰瓷没法处理
    t_ub_index = std::min(t_ub_index + 2, dp_end_index);

    if (obs_s > dp_s) //决策为减速避让
    {
      for (int i = t_lb_index; i < t_ub_index; i++) {
        double dp_t = dp_speed_points_[i].t;
        // % 在t_lb_index:t_ub_index的区间上 s的上界不可以超过障碍物st斜直线
        convex_s_ub_[i] =
            std::min(convex_s_ub_[i],
                     obs.in_point.s + obs_speed * (dp_t - obs.in_point.t));
      }
    } else {
      for (int i = t_lb_index; i < t_ub_index; i++) {
        double dp_t = dp_speed_points_[i].t;
        // 在t_lb_index:t_ub_index的区间上 s的下界不能小于障碍物st斜直线
        convex_s_lb_[i] =
            std::max(convex_s_lb_[i],
                     obs.in_point.s + obs_speed * (dp_t - obs.in_point.t));
      }
    }
  }
}

int SpeedTimeGraph::FindDpMatchIndex(double t) {
  // t_in,t_out对应的动态规划点
  int index = 0;
  if (t <= dp_speed_points_.front().t)
    return 0;
  if (t >= dp_speed_points_.back().t)
    return dp_speed_points_.size() - 1;
  for (int j = 0; j < dp_speed_points_.size() - 1; j++) {
    if (t >= dp_speed_points_[j].t &&
        t < dp_speed_points_[j + 1].t) { //% 否则遍历dp_speed_t
                      //找到与obs_st_t_in_set(i)最近的点的编号
      index = j;
      break;
    }
  }
  return index;
}

// 5.二次规划
bool SpeedTimeGraph::SpeedQuadraticProgramming() {
  /*
          %速度二次规划
      % 输入：规划起点plan_start_s_dot,plan_start_s_dot2
      %       动态规划结果dp_speed_s,dp_speed_t
      %       凸空间约束 s_lb,s_ub,s_dot_lb,s_dot_ub
      %
     加速度代价权重，推荐速度代价权重，jerk代价权重w_cost_s_dot2,w_cost_v_ref,w_cost_jerk
      %       参考速度speed_reference
      % 输出：速度曲线qp_s_init,qp_s_dot_init,qp_s_dot2_init,relative_time_init
  */

  int n = dp_speed_points_.size(); //??输出

  // int qp_size = dp_speed_points_.size();
  // Hissen矩阵 3n*3n
  Eigen::SparseMatrix<double> A_ref(3 * n, 3 * n);
  Eigen::SparseMatrix<double> A_dds_dt(3 * n, 3 * n);
  Eigen::SparseMatrix<double> A_jerk(n - 1, 3 * n);
  Eigen::SparseMatrix<double> H(3 * n, 3 * n);

  // f矩阵 3n*1
  Eigen::VectorXd f = Eigen::VectorXd::Zero(3 * n);

  //不等式约束,允许倒车
  Eigen::SparseMatrix<double> A(n - 1, 3 * n);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(n - 1);

  //等式约束，连续性
  Eigen::SparseMatrix<double> Aeq(2 * n - 2, 3 * n);
  // Eigen::SparseMatrix<double> Aeq_tran(3 * n, 2 * n - 2);
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(2 * n - 2);

  //上下边界约束，凸空间
  Eigen::SparseMatrix<double> A_lu(3 * n, 3 * n);
  A_lu.setIdentity();
  Eigen::VectorXd lb = Eigen::VectorXd::Zero(3 * n);
  Eigen::VectorXd ub = Eigen::VectorXd::Zero(3 * n);

  Eigen::SparseMatrix<double> A_merge(n - 1 + 2 * n - 2 + 3 * n, 3 * n);

  Eigen::VectorXd lb_merge = Eigen::VectorXd::Zero(n - 1 + 2 * n - 2 + 3 * n);
  Eigen::VectorXd ub_merge = Eigen::VectorXd::Zero(n - 1 + 2 * n - 2 + 3 * n);

  //生成H
  for (int i = 0; i < n; i++) {
    A_ref.insert(3 * i + 1, 3 * i + 1) = 1;
    A_dds_dt.insert(3 * i + 2, 3 * i + 2) = 1;
  }

  // Eigen::MatrixXd A_jerk_sub(6, 1);
  // A_jerk_sub << 0, 0, 1, 0, 0, -1;

  //??
  for (int i = 0; i < n - 1; i++) {
    A_jerk.insert(i, 3 * i + 2) = 1;
    A_jerk.insert(i, 3 * (i + 1) + 2) = -1;
  }

  //为什么二次规划，没有障碍物代价
  H = emplaner_conf_["speed_qp_cost_v_ref"] * (A_ref.transpose() * A_ref) +
      emplaner_conf_["speed_qp_cost_dds_dt"] * (A_dds_dt.transpose() * A_dds_dt) +
      emplaner_conf_["speed_qp_cost_jerk"] * (A_jerk.transpose() * A_jerk);
  H = 2.0 * H;

  //生成f
  for (int i = 0; i < n; i++) {
    f(3 * i + 1) =
        -2.0 * emplaner_conf_["speed_qp_cost_v_ref"] * emplaner_conf_["ref_speed"];
  }
  int index_start = 0;
  // 不等式约束，生成A 不允许倒车 si+1-si>0
  for (int i = 0; i < n - 1; i++) {
    A_merge.insert(i, 3 * i) = 1;
    A_merge.insert(i, 3 * i + 3) = -1;
  }

  //生成b,b为0列向量

  //等式约束，生成Aeq,连续性约束。学习加加速度算法的原理
  double dt = emplaner_conf_["speed_dp_sample_t"];

  // Eigen::MatrixXd A_sub(6, 2);
  // A_sub << 1, 0, dt, 1, (1 / 3) * pow(dt, 2), (1 / 2) * dt, -1, 0, 0, -1,
  //     (1 / 6) * pow(dt, 2), dt / 2;
  index_start = n - 1;
  for (int i = 0; i < n - 1; i++) {
    double row = index_start + i * 2;
    double col = i * 3;
    A_merge.insert(row, col) = 1;
    A_merge.insert(row, col + 1) = dt;
    A_merge.insert(row, col + 2) = (1.0 / 3) * pow(dt, 2);
    A_merge.insert(row, col + 3) = -1;
    A_merge.insert(row, col + 4) = 0;
    A_merge.insert(row, col + 5) = (1.0 / 6) * pow(dt, 2);

    A_merge.insert(row + 1, col) = 0;
    A_merge.insert(row + 1, col + 1) = 1;
    A_merge.insert(row + 1, col + 2) = (1.0 / 2) * dt;
    A_merge.insert(row + 1, col + 3) = 0;
    A_merge.insert(row + 1, col + 4) = -1;
    A_merge.insert(row + 1, col + 5) = dt / 2;
  }

  index_start = n - 1 + 2 * n - 2;
  for (int i = 0; i < n; i++) {
    int row = index_start + 3 * i;
    int col = 3 * i;
    A_merge.insert(row, col) = 1;
    A_merge.insert(row + 1, col + 1) = 1;
    A_merge.insert(row + 2, col + 2) = 1;
  }

  // beq=0

  //凸空间约束 生成lb,ub
  for (int i = 1; i < n; i++) {
    lb(3 * i) = convex_s_lb_(i);
    lb(3 * i + 1) = convex_ds_dt_lb_(i);
    lb(3 * i + 2) = -6; //最小加速度

    ub(3 * i) = convex_s_ub_(i);
    ub(3 * i + 1) = convex_ds_dt_ub_(i);
    ub(3 * i + 2) = 4; //最大加速
  }

  //起点约束端点约束
  lb(0) = 0;
  lb(1) = st_plan_start_.ds_dt;
  lb(2) = st_plan_start_.dds_dt;
  ub(0) = lb(0);
  ub(1) = lb(1);
  ub(2) = lb(2);

  lb_merge.block(0, 0, n - 1, 1) = Eigen::VectorXd::Ones(n - 1) * -DBL_MAX;
  lb_merge.block(n - 1, 0, 2 * n - 2, 1) = beq;
  lb_merge.block(n - 1 + 2 * n - 2, 0, 3 * n, 1) = lb;

  ub_merge.block(0, 0, n - 1, 1) = b;
  ub_merge.block(n - 1, 0, 2 * n - 2, 1) = beq;
  ub_merge.block(n - 1 + 2 * n - 2, 0, 3 * n, 1) = ub;

  OsqpEigen::Solver solver;
  solver.settings()->setWarmStart(true);
  solver.data()->setNumberOfVariables(3 * n); // x
  solver.data()->setNumberOfConstraints(n - 1 + 2 * n - 2 + 3 * n); // A
  solver.data()->setHessianMatrix(H);
  solver.data()->setGradient(f);
  solver.data()->setLinearConstraintsMatrix(A_merge);
  solver.data()->setLowerBound(lb_merge);
  solver.data()->setUpperBound(ub_merge);

  if (!solver.initSolver())
    return 1;
  if (!solver.solve())
    return 1;

  Eigen::VectorXd qp_solution(3 * n);
  qp_solution = solver.getSolution();

  qp_speed_points_.resize(n);
  for (int i = 0; i < n; i++) {
    qp_speed_points_[i].s = qp_solution(3 * i);
    if (qp_speed_points_[i].s < 0)
    // 理论上s的最小值应该为0，不应该小于0，在约束里面也有
    // 但可能因为我double的原因，这里计算出来的值是-1*e-6很接近0的值，但是会导致后面的拼接出问题
      qp_speed_points_[i].s  = 0; 
    qp_speed_points_[i].ds_dt = qp_solution(3 * i + 1);
    if (qp_speed_points_[i].ds_dt < 0)
    // 不允许后退，理论上速度也不应该小于0
      qp_speed_points_[i].ds_dt = 0;
    qp_speed_points_[i].dds_dt = qp_solution(3 * i + 2);
    qp_speed_points_[i].t = i * dt;
  }
  return 0;
}

void SpeedTimeGraph::SpeedQpInterpolation() //点的个数401
{
  /*
      %该函数将增密s s_dot s_dot2
      %
     为什么需要增密，因为控制的执行频率是规划的10倍，轨迹点如果不够密，必然会导致规划效果不好
      % 但是若在速度二次规划中点取的太多，会导致二次规划的矩阵规模太大计算太慢
      % 所以方法是在二次规划中选取少量的点优化完毕后，在用此函数增密
  */

  //是否排除空数据
  double dt = emplaner_conf_["speed_qp_interp_dt"];
  double t_qp = qp_speed_points_.back().t;
  int n = t_qp / dt + 1;
  qp_speed_points_dense_.resize(n);

  for (int i = 0; i < n; i++) {
    double cur_t = i * dt;
    int j = 0;
    for (j = 0; j < qp_speed_points_.size() - 1; j++) {
      if (qp_speed_points_[j].t <= cur_t && qp_speed_points_[j + 1].t > cur_t)
        break;
    }
    double dt_2pre = cur_t - qp_speed_points_[j].t;
    // 下面的方式会导致有1的那两项转化为整数，因为类型转换取决于第一项
    // 要么使用1.0要么除以6
    // qp_speed_points_dense_[i].s =
    //     qp_speed_points_[j].s + qp_speed_points_[j].ds_dt * dt_2pre +
    //     (1 / 3) * qp_speed_points_[j].dds_dt * pow(dt_2pre, 2) +
    //     (1 / 6) * qp_speed_points_[j + 1].dds_dt * pow(dt_2pre, 2);
    qp_speed_points_dense_[i].s =
        qp_speed_points_[j].s + qp_speed_points_[j].ds_dt * dt_2pre +
        qp_speed_points_[j].dds_dt * pow(dt_2pre, 2) / 3 +
        qp_speed_points_[j + 1].dds_dt * pow(dt_2pre, 2) / 6;
    qp_speed_points_dense_[i].ds_dt =
        qp_speed_points_[j].ds_dt + 0.5 * qp_speed_points_[j].dds_dt * dt_2pre + 
        0.5 * qp_speed_points_[j + 1].dds_dt * dt_2pre;
    qp_speed_points_dense_[i].dds_dt =
        qp_speed_points_[j].dds_dt +
        (qp_speed_points_[j + 1].dds_dt - qp_speed_points_[j].dds_dt) *
            dt_2pre / (qp_speed_points_[j + 1].t - qp_speed_points_[j].t);
    qp_speed_points_dense_[i].t = cur_t;
  }
}

void SpeedTimeGraph::PathAndSpeedMerge(const double &plan_time) {
  /*
  该函数将合并path和speed
  % 由于path 是 61个点，speed 有
  601个点，合并后，path和speed有601个点，因此需要做插值
  */
  int n = qp_speed_points_dense_.size();
  auto planning_path_points = planning_path_.reference_points();
  trajectory_points_.resize(n);
  for (int i = 0; i < n; i++) {
    trajectory_points_[i].t = plan_time + qp_speed_points_dense_[i].t;
    trajectory_points_[i].v = qp_speed_points_dense_[i].ds_dt;
    trajectory_points_[i].a = qp_speed_points_dense_[i].dds_dt;

    //查找s对应到sl的index
    int j = 0;
    for (j = 0; j < sl_planning_path_.size() - 1; j++) {
      if (sl_planning_path_[j].s <= qp_speed_points_dense_[i].s &&
          sl_planning_path_[j + 1].s > qp_speed_points_dense_[i].s)
        break;
    }
    //线性插值

    double k = (qp_speed_points_dense_[i].s - sl_planning_path_[j].s) /
               (sl_planning_path_[j + 1].s - sl_planning_path_[j].s);

    trajectory_points_[i].x =
        planning_path_points[j].x +
        k * (planning_path_points[j + 1].x - planning_path_points[j].x);
    trajectory_points_[i].y =
        planning_path_points[j].y +
        k * (planning_path_points[j + 1].y - planning_path_points[j].y);
    trajectory_points_[i].heading = planning_path_points[j].heading +
                                    k * (planning_path_points[j + 1].heading -
                                         planning_path_points[j].heading);
    trajectory_points_[i].kappa =
        planning_path_points[j].kappa +
        k * (planning_path_points[j + 1].kappa - planning_path_points[j].kappa);
  }

  trajectory_.set_trajectory_points(trajectory_points_);
}

const Trajectory SpeedTimeGraph::trajectory() const { return trajectory_; }

const std::vector<VirtualObs> SpeedTimeGraph::virtual_obstacles() const {
  return virtual_obstacles_;
} //虚拟障碍物的xy坐标

std::vector<STLine> SpeedTimeGraph::st_obstacles() { return st_obstacles_; }

const std::vector<STPoint> SpeedTimeGraph::dp_speed_points() const {
  return dp_speed_points_;
}

const std::vector<STPoint> SpeedTimeGraph::qp_speed_points() const {
  return qp_speed_points_;
}

const std::vector<STPoint> SpeedTimeGraph::qp_speed_points_dense() const {
  return qp_speed_points_dense_;
}