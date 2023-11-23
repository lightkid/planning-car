#include "hybrid_a_star/hybrid_a_star.h"

HybridAStar::HybridAStar(ros::NodeHandle &nh) : nh_(nh) {
  map_.reset(new CostMap(nh_, "/map"));
  while (!map_->hasMap() && ros::ok()) {
    ros::spinOnce(); // wait for map
  }
  ROS_INFO("planner get map!");
  nh.param("wheel_base", wheel_base_, 3.0);
  nh.param("width", width_, 2.0);
  nh.param("length", length_, 4.0);
  nh.param("steering_angle", steering_angle_, 15.0);
  nh.param("steering_angle_discrete_num", steering_angle_discrete_num_, 1);
  nh.param("segment_length", segment_length_, 1.6);
  nh.param("segment_length_discrete_num", segment_length_discrete_num_, 8);
  nh.param("grid_size_xy", grid_size_xy_, 5);
  nh.param("grid_size_phi", grid_size_phi_, 72);
  nh.param("steering_penalty", steering_penalty_, 1.05);
  nh.param("reversing_penalty", reversing_penalty_, 2.0);
  nh.param("steering_change_penalty", steering_change_penalty_, 1.5);
  nh.param("shot_distance", shot_distance_, 5.0);
  steering_radian_ = steering_angle_ * M_PI / 180.0; // angle to radian
  steering_radian_step_size_ = steering_radian_ / steering_angle_discrete_num_;
  move_step_size_ = segment_length_ / segment_length_discrete_num_;
  grid_resolution_xy_ = map_->resolution() / grid_size_xy_;
  grid_resolution_phi_ = M_PI * 2.0 / grid_size_phi_;
  map_->setShape(width_, length_);
  // collision_radius_ =
  //     std::ceil(std::sqrt(width_ * width_ + length_ * length_) * 0.5);
  //记录车的四个角
  /*
   * 0  ^x 3
   * <y-|--
   * 1  |  2
   */
  // car_corner_.block<2, 1>(0, 0) = Eigen::Vector2d(length_ / 2, width_ / 2);
  // car_corner_.block<2, 1>(2, 0) = Eigen::Vector2d(-length_ / 2, width_ / 2);
  // car_corner_.block<2, 1>(4, 0) = Eigen::Vector2d(-length_ / 2, -width_ / 2);
  // car_corner_.block<2, 1>(6, 0) = Eigen::Vector2d(length_ / 2, -width_ / 2);
  std::cout << "wheel_base: " << wheel_base_ << std::endl
            << "width: " << width_ << std::endl
            << "length: " << length_ << std::endl
            << "steering_angle: " << steering_angle_ << std::endl
            << "steering_angle_discrete_num: " << steering_angle_discrete_num_
            << std::endl
            << "segment_length: " << segment_length_ << std::endl
            << "segment_length_discrete_num: " << segment_length_discrete_num_
            << std::endl
            << "move_step_size: " << move_step_size_ << std::endl
            << "grid_size_xy: " << grid_size_xy_ << std::endl
            << "grid_size_phi: " << grid_size_phi_ << std::endl
            << "steering_penalty: " << steering_penalty_ << std::endl
            << "reversing_penalty: " << reversing_penalty_ << std::endl
            << "steering_change_penalty: " << steering_change_penalty_
            << std::endl
            << "shot_distance: " << shot_distance_ << std::endl;
  // << "collision_radius: " << collision_radius_ << std::endl;

  rs_path_ptr_ =
      std::make_shared<RSPath>(wheel_base_ / std::tan(steering_radian_));

  // test
  nbr_vis_pub_ = nh_.advertise<nav_msgs::Path>("search_tree", 10);
  shape_vis_pub_ = nh.advertise<visualization_msgs::Marker>("shape", 10);
  select_point_pub_ = nh.advertise<nav_msgs::Path>("select_point", 10);
  select_points_.header.frame_id = "map";

  global_path_pub_ = nh_.advertise<nav_msgs::Path>("global_path", 2);
  goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 10, &HybridAStar::goalCallback, this);
  ROS_INFO("planner is ready!");
}

void HybridAStar::goalCallback(
    const geometry_msgs::PoseStamped::ConstPtr &msg) {
  double x1 = (*msg).pose.orientation.x;
  double y1 = (*msg).pose.orientation.y;
  double z1 = (*msg).pose.orientation.z;
  double w1 = (*msg).pose.orientation.w;
  double selfAngle1 = std::atan2(2 * (w1 * z1 + x1 * y1),
                                 1 - 2 * (y1 * y1 + z1 * z1)); // 偏航角
  if (isPose_) {
    pose_(0) = msg->pose.position.x;
    pose_(1) = msg->pose.position.y;
    pose_(2) = selfAngle1;
    visualization_msgs::Marker shape;
    shape.header.frame_id = "map";
    shape.header.stamp = ros::Time::now();
    shape.ns = "shape";
    shape.id = 0;
    shape.type = visualization_msgs::Marker::CUBE;
    shape.action = visualization_msgs::Marker::ADD;
    shape.pose = (*msg).pose;
    shape.scale.x = 4;
    shape.scale.y = 2;
    shape.color.a = 0.5;
    shape.color.r = 0.5;
    shape_vis_pub_.publish(shape);
  } else {
    goal_(0) = msg->pose.position.x;
    goal_(1) = msg->pose.position.y;
    goal_(2) = selfAngle1;
    // Eigen::Vector2i start, goal;
    // map_->posToIdx(pose_, start);
    // map_->posToIdx(goal_, goal);
    // std::cout << "start: " << start.transpose() << std::endl;
    // std::cout << "goal: " << goal.transpose() << std::endl;
    nav_msgs::Path global_path;
    select_points_.poses.clear();
    auto res = searchPath(pose_, goal_, global_path);
    if (res.IsOK()) {
      pathVisualization(global_path);
      ROS_INFO("Global path search successed,%s", res.error_msg().c_str());
    } else {
      ROS_ERROR("Global path search failed,%s", res.error_msg().c_str());
    }
  }
  isPose_ = !isPose_;
}
void HybridAStar::pathVisualization(const nav_msgs::Path &path) {
  global_path_pub_.publish(path);
}
ErrorInfo HybridAStar::searchPath(const Eigen::Vector3d &start_pose,
                                  const Eigen::Vector3d &goal_pose,
                                  nav_msgs::Path &path) {
  path.poses.clear();
  search_tree_.clear();
  auto compare = [&](const StateNode::Ptr &n1, const StateNode::Ptr &n2) {
    return n1->g_score_ + n1->h_score_ > n2->g_score_ + n2->h_score_;
  };
  std::priority_queue<StateNode::Ptr, std::vector<StateNode::Ptr>,
                      decltype(compare)>
      openlist(compare);
  // create start node
  int start_adr = toAdrHighRes(start_pose);
  StateNode::Ptr start_node_ptr = std::make_shared<StateNode>(start_adr);
  start_node_ptr->pose_ = start_pose;
  start_node_ptr->steering_ = 0.0;
  start_node_ptr->direction_ = StateNode::Direction::NO;
  start_node_ptr->search_state_ = StateNode::SearchState::OPEN;
  start_node_ptr->intermediate_states_.emplace_back(start_pose);
  start_node_ptr->g_score_ = 0.0;

  // create end node
  StateNode::Ptr end_node_ptr;
  int goal_adr = toAdrHighRes(goal_pose);
  StateNode::Ptr goal_node_ptr = std::make_shared<StateNode>(goal_adr);
  goal_node_ptr->pose_ = goal_pose;
  goal_node_ptr->steering_ = 0.0;
  goal_node_ptr->direction_ = StateNode::Direction::NO;
  // goal_node_ptr->search_state_ = StateNode::SearchState::NOT_HANDLED;
  // goal_node_ptr->intermediate_states_.emplace_back(goal_pose);
  // goal_node_ptr->g_score_ = 0.0;

  // begin search
  openlist.push(start_node_ptr);

  search_tree_[start_adr] = start_node_ptr; // open & close
  // std::cout << "search tree size: " << search_tree_.size() << std::endl;
  std::vector<StateNode::Ptr> neighbor_nodes_ptr;
  bool success = false;
  unsigned int count = 0;
  while (!openlist.empty() && ros::ok()) {
    std::cout << "openlist size: " << openlist.size() << std::endl;
    std::cout << "search tree size: " << search_tree_.size() << std::endl;
    auto current_ptr = openlist.top();
    openlist.pop();
    auto iter = search_tree_.find(current_ptr->adr_); // O(logn)
    if (iter == search_tree_.end()) {
      // error
    }
    iter->second->search_state_ = StateNode::SearchState::CLOSED;

    // is the goal? //TODO
    // if (toAdrHighRes(current_ptr->pose_) == goal_adr)
    if ((current_ptr->pose_.head(2) - goal_node_ptr->pose_.head(2)).norm() <=
        shot_distance_) {
      std::cout << "so close" << std::endl;
      double rs_length = 0.0;
      // if rs曲线链接无碰撞
      if (AnalyticExpansions(current_ptr, goal_node_ptr, rs_length)) {
        std::cout << "success" << std::endl;
        end_node_ptr = goal_node_ptr;
        success = true;
        break;
      }
    }

    // get neighbors
    getNeighbors(current_ptr, neighbor_nodes_ptr);
    std::cout << "nbr size: " << neighbor_nodes_ptr.size() << std::endl;
    for (auto nbr_ptr : neighbor_nodes_ptr) {
      nav_msgs::Path branch;
      branch.header.frame_id = "map";
      branch.header.stamp = ros::Time::now();
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = current_ptr->pose_.x();
      pose.pose.position.y = current_ptr->pose_.y();
      pose.pose.orientation.w = cos(current_ptr->pose_.z() * 0.5);
      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = sin(current_ptr->pose_.z() * 0.5);
      branch.poses.push_back(pose);
      select_points_.poses.push_back(pose);
      select_point_pub_.publish(select_points_);
      for (auto pos : nbr_ptr->intermediate_states_) {
        pose.pose.position.x = pos.x();
        pose.pose.position.y = pos.y();
        pose.pose.orientation.w = cos(pos.z() * 0.5);
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = sin(pos.z() * 0.5);
        branch.poses.push_back(pose);
      }
      nbr_vis_pub_.publish(branch);
      // compute move cost
      double move_cost = getMoveCost(current_ptr, nbr_ptr);
      double h_score = getHeuristic(current_ptr, goal_node_ptr); // TODO
      auto iter_nbr = search_tree_.find(nbr_ptr->adr_);
      if (iter_nbr == search_tree_.end()) { // not handled
        // compute nbr h_score

        nbr_ptr->g_score_ = current_ptr->g_score_ + move_cost;
        nbr_ptr->h_score_ = h_score;
        nbr_ptr->parent_ = current_ptr;
        nbr_ptr->search_state_ = StateNode::SearchState::OPEN;
        openlist.push(nbr_ptr);
        // std::cout << "open list size +1" << std::endl;
        search_tree_[nbr_ptr->adr_] = nbr_ptr;
      } else {
        if (iter_nbr->second->search_state_ == StateNode::SearchState::CLOSED) {
          continue;
        } else /*if (iter_nbr->second->search_state_ ==
                  StateNode::SearchState::OPEN)*/
        {
          double new_g = current_ptr->g_score_ + move_cost;
          if (new_g < iter_nbr->second->g_score_) {
            // iter_nbr->second->g_score_ = new_g;
            // iter_nbr->second->h_score_ = h_score;
            // iter_nbr->second->parent_ = current_ptr;

            nbr_ptr->g_score_ = new_g;
            nbr_ptr->h_score_ = h_score;
            nbr_ptr->parent_ = current_ptr;
            nbr_ptr->search_state_ = StateNode::SearchState::OPEN;
            // nbr_ptr->pose_=
            iter_nbr->second = nbr_ptr;
          }
        }
      }
    }
    if (++count > 50000) {
      // error
      break;
    }
  }
  // if not the goal
  if (!success) {
    // error
    return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR,
                     "Valid global path not found");
  }
  std::cout << "success" << std::endl;
  // get path
  path.poses.clear();
  path.header.frame_id = "map";
  // start->current-RS->goal
  // VectorVec3d path;
  std::vector<StateNode::Ptr> temp_nodes;
  while (end_node_ptr != nullptr) {
    temp_nodes.emplace_back(end_node_ptr);
    end_node_ptr = end_node_ptr->parent_;
  }
  std::reverse(temp_nodes.begin(), temp_nodes.end());
  for (const auto &node : temp_nodes) {
    for (const auto &pos : node->intermediate_states_) {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = pos.x();
      pose.pose.position.y = pos.y();
      pose.pose.orientation.w = cos(pos.z() * 0.5);
      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = sin(pos.z() * 0.5);
      path.poses.push_back(pose);
    }
  }
  return ErrorInfo(ErrorCode::OK,
                   "Global path length: " + std::to_string(path.poses.size()));
}
double HybridAStar::getMoveCost(const StateNode::Ptr &current_node_ptr,
                                const StateNode::Ptr &neighbor_node_ptr) {
  double g;
  if (neighbor_node_ptr->direction_ == StateNode::FORWARD) {
    if (neighbor_node_ptr->steering_ != current_node_ptr->steering_) {
      if (static_cast<int>(neighbor_node_ptr->steering_) == 0) {
        g = segment_length_ * steering_change_penalty_;
      } else {
        g = segment_length_ * steering_change_penalty_ * steering_penalty_;
      }
    } else {
      if (static_cast<int>(neighbor_node_ptr->steering_) == 0) {
        g = segment_length_;
      } else {
        g = segment_length_ * steering_penalty_;
      }
    }
  } else {
    if (neighbor_node_ptr->steering_ != current_node_ptr->steering_) {
      if (static_cast<int>(neighbor_node_ptr->steering_) == 0) {
        g = segment_length_ * steering_change_penalty_ * reversing_penalty_;
      } else {
        g = segment_length_ * steering_change_penalty_ * steering_penalty_ *
            reversing_penalty_;
      }
    } else {
      if (static_cast<int>(neighbor_node_ptr->steering_) == 0) {
        g = segment_length_ * reversing_penalty_;
      } else {
        g = segment_length_ * steering_penalty_ * reversing_penalty_;
      }
    }
  }

  return g;
  // // 前进距离
  // double g = neighbor_node_ptr->intermediate_states_.size() *
  // move_step_size_;
  // // 打角
  // double s = abs(neighbor_node_ptr->steering_);
  // // 打角变化
  // double ds = abs(neighbor_node_ptr->steering_ -
  // current_node_ptr->steering_); g *= (1 + steering_penalty_ * s) * (1 +
  // steering_change_penalty_ * ds); if (neighbor_node_ptr->direction_ !=
  // StateNode::FORWARD) {
  //   g *= reversing_penalty_;
  // }
  // return g;
}
double HybridAStar::getHeuristic(const StateNode::Ptr &current_node_ptr,
                                 const StateNode::Ptr &terminal_node_ptr) {
  double h;
  h = (current_node_ptr->pose_.head(2) - terminal_node_ptr->pose_.head(2))
          .lpNorm<1>();

  if (h < 3.0 * shot_distance_) {
    h = rs_path_ptr_->Distance(
        current_node_ptr->pose_.x(), current_node_ptr->pose_.y(),
        current_node_ptr->pose_.z(), terminal_node_ptr->pose_.x(),
        terminal_node_ptr->pose_.y(), terminal_node_ptr->pose_.z());
  }

  return h;
}
void HybridAStar::getNeighbors(const StateNode::Ptr &current_node,
                               std::vector<StateNode::Ptr> &neighbors_node) {
  neighbors_node.clear();
  // for each sampled steering angle
  // perform forward and backward integral
  // check collision for each intermediate iterate
  // if collision push the last collision-free state into
  // neighbors_node（有障碍这条不要）
  // std::cout << "pos: " << current_node->pose_.transpose() << std::endl;
  for (int i = -steering_angle_discrete_num_; i <= steering_angle_discrete_num_;
       ++i) {
    VectorVec3d intermediate_state;
    bool has_obstacle = false;
    double x = current_node->pose_.x();
    double y = current_node->pose_.y();
    double theta = current_node->pose_.z();

    const double phi = i * steering_radian_step_size_;

    // forward
    for (int j = 1; j <= segment_length_discrete_num_; j++) {
      dynamicModel(move_step_size_, phi, x, y, theta);
      intermediate_state.emplace_back(Vec3d(x, y, theta));

      if (map_->isShapeCollision(x, y, theta)) {
        has_obstacle = true;
        break;
      }
    }

    int grid_adr = toAdrHighRes(intermediate_state.back());
    Eigen::Vector2d pos = intermediate_state.back().head(2);
    // std::cout << "forward: " << pos.transpose()
    //           << " isInMap: " << map_->isInMap(pos) << std::endl;
    if (/*map_->isInMap(pos) &&*/ !has_obstacle) {
      StateNode::Ptr neighbor_forward_node_ptr =
          std::make_shared<StateNode>(grid_adr);
      neighbor_forward_node_ptr->intermediate_states_ = intermediate_state;
      neighbor_forward_node_ptr->pose_ = intermediate_state.back();
      neighbor_forward_node_ptr->steering_ = phi;
      neighbor_forward_node_ptr->direction_ = StateNode::FORWARD;
      neighbors_node.push_back(neighbor_forward_node_ptr);
      // std::cout << "nbr size1: " << neighbors_node.size() << std::endl;
    }

    // backward
    has_obstacle = false;
    intermediate_state.clear();
    x = current_node->pose_.x();
    y = current_node->pose_.y();
    theta = current_node->pose_.z();
    for (int j = 1; j <= segment_length_discrete_num_; j++) {
      dynamicModel(-move_step_size_, phi, x, y, theta);
      intermediate_state.emplace_back(Vec3d(x, y, theta));

      if (map_->isShapeCollision(x, y, theta)) {
        has_obstacle = true;
        break;
      }
    }

    grid_adr = toAdrHighRes(intermediate_state.back());
    pos = intermediate_state.back().head(2);
    if (/*map_->isInMap(pos) &&*/ !has_obstacle) {
      StateNode::Ptr neighbor_forward_node_ptr =
          std::make_shared<StateNode>(grid_adr);
      neighbor_forward_node_ptr->intermediate_states_ = intermediate_state;
      neighbor_forward_node_ptr->pose_ = intermediate_state.back();
      neighbor_forward_node_ptr->steering_ = phi;
      neighbor_forward_node_ptr->direction_ = StateNode::BACKWARD;
      neighbors_node.push_back(neighbor_forward_node_ptr);
      // std::cout << "nbr size2: " << neighbors_node.size() << std::endl;
    }
  }
  // std::cout << "nbr size: " << neighbors_node.size() << std::endl;
}
bool HybridAStar::AnalyticExpansions(const StateNode::Ptr &current_node_ptr,
                                     const StateNode::Ptr &goal_node_ptr,
                                     double &length) {
  VectorVec3d rs_path_poses = rs_path_ptr_->GetRSPath(
      current_node_ptr->pose_, goal_node_ptr->pose_, move_step_size_, length);

  // for (const auto &pose : rs_path_poses) {
  //   // Eigen::Vector2d pos = pose.head(2);
  //   if (/*map_->isShapeInMap(pose.x(), pose.y(), pose.z()) ||*/
  //       !map_->isShapeCollision(pose.x(), pose.y(), pose.z())) {
  //     return false;
  //   };
  // }

  goal_node_ptr->intermediate_states_ = rs_path_poses;
  goal_node_ptr->parent_ = current_node_ptr;

  auto begin = goal_node_ptr->intermediate_states_.begin();
  goal_node_ptr->intermediate_states_.erase(begin); //中间状态不能以存上一个节点

  return true;
}
void HybridAStar::dynamicModel(const double &step, const double &steer,
                               double &x, double &y, double &phi) {
  x += step * std::cos(phi);
  y += step * std::sin(phi);
  phi = mod2Pi(phi + step * std::tan(steer) / wheel_base_);
}
// bool HybridAStar::checkCollision(const double &x, const double &y,
//                                  const double &theta) {
//   return false;
//   //对车中心周围的occ grid和车做矩形碰撞检测
//   Eigen::Vector2i car_idx;
//   map_->posToIdx(Eigen::Vector2d(x, y), car_idx);
//   double r = map_->resolution() * 0.5;

//   //车的四个角
//   Mat2d R;
//   R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
//   Eigen::Matrix<double, 8, 1> transformed_vehicle_shape;
//   double min_x_car = -1000000, max_x_car = 1000000, min_y_car = -1000000,
//          max_y_car = 1000000;
//   for (unsigned int i = 0; i < 4u; ++i) {
//     transformed_vehicle_shape.segment<2>(i * 2) =
//         R * car_corner_.segment<2>(i * 2) + Eigen::Vector2d(x, y);
//     min_x_car = std::min(min_x_car, transformed_vehicle_shape(i * 2));
//     min_y_car = std::min(min_y_car, transformed_vehicle_shape(i * 2 + 1));
//     max_x_car = std::max(max_x_car, transformed_vehicle_shape(i * 2));
//     max_y_car = std::max(max_y_car, transformed_vehicle_shape(i * 2 + 1));
//   }

//   for (int dx = -collision_radius_; dx <= collision_radius_; ++dx) {
//     for (int dy = -collision_radius_; dy <= collision_radius_; ++dy) {
//       Eigen::Vector2i idx = car_idx;
//       idx(0) += dx;
//       idx(1) += dy;
//       if (map_->isOcc(idx)) {
//         Eigen::Vector2d cen_pos;
//         map_->idxToPos(idx, cen_pos);
//         double min_x = cen_pos(0) - r, max_x = cen_pos(0) + r,
//                min_y = cen_pos(1) - r, max_y = cen_pos(1) + r;
//         if (!(((min_x > max_x_car) || (max_x < min_x_car)) &&
//               ((min_y > max_y_car) || (max_y < min_y_car)))) { // overlap
//           return true;
//         }
//       }
//     }
//   }
//   return false;
// }
double HybridAStar::mod2Pi(const double &phi) {
  double v = fmod(phi, 2 * M_PI);
  if (v < -M_PI) {
    v += 2.0 * M_PI;
  } else if (v > M_PI) {
    v -= 2.0 * M_PI;
  }
  return v;
}
Eigen::Vector3i HybridAStar::posToIdxHighRes(const Eigen::Vector3d &pos) {
  Eigen::Vector3i idx;
  idx(0) = floor((pos(0) - map_->map_origin()(0)) / grid_resolution_xy_);
  idx(1) = floor((pos(1) - map_->map_origin()(1)) / grid_resolution_xy_);
  idx(2) = floor((pos(2) - (-M_PI)) / grid_resolution_phi_);
  return idx;
}
void HybridAStar::idxToPosHighRes(const Eigen::Vector3i &idx,
                                  Eigen::Vector3d &pos) {}
int HybridAStar::toAdrHighRes(const Eigen::Vector3i &idx) {
  return idx(0) * (map_->height() * grid_size_xy_) * grid_size_phi_ +
         idx(1) * grid_size_phi_ + idx(2);
}
int HybridAStar::toAdrHighRes(const Eigen::Vector3d &pos) {
  Eigen::Vector3i idx;
  idx(0) = floor((pos(0) - map_->map_origin()(0)) / grid_resolution_xy_);
  idx(1) = floor((pos(1) - map_->map_origin()(1)) / grid_resolution_xy_);
  idx(2) = floor((pos(2) - (-M_PI)) / grid_resolution_phi_);
  return idx(0) * (map_->height() * grid_size_xy_) * grid_size_phi_ +
         idx(1) * grid_size_phi_ + idx(2);
}
Eigen::Vector3i HybridAStar::adrToIdxHighRes(const int &adr) {}