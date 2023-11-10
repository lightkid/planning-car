#include "a_star/a_star.h"

AStar::AStar(ros::NodeHandle &nh) : nh_(nh) {
  map_.reset(new CostMap(nh_, "/map"));
  while (!map_->hasMap() && ros::ok()) {
    ros::spinOnce(); // wait for map
  }
  ROS_INFO("planner get map!");
  global_path_pub_ = nh_.advertise<nav_msgs::Path>("global_path", 2);
  pose_sub_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
      "/initialpose", 10, &AStar::poseCallback, this);
  goal_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 10, &AStar::goalCallback, this);
  ROS_INFO("planner is ready!");
}
void AStar::poseCallback(
    const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
  // pose_(0) = msg->pose.pose.position.x;
  // pose_(1) = msg->pose.pose.position.y;
}
void AStar::goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  if (isPose_) {
    pose_(0) = msg->pose.position.x;
    pose_(1) = msg->pose.position.y;
  } else {
    goal_(0) = msg->pose.position.x;
    goal_(1) = msg->pose.position.y;
    Eigen::Vector2i start, goal;
    map_->posToIdx(pose_, start);
    map_->posToIdx(goal_, goal);
    std::cout << "start: " << start.transpose() << std::endl;
    std::cout << "goal: " << goal.transpose() << std::endl;
    nav_msgs::Path global_path;
    auto res = searchPath(start, goal, global_path);
    if (res.IsOK()) {
      pathVisualization(global_path);
      ROS_INFO("Global path search successed,%s", res.error_msg().c_str());
    } else {
      ROS_ERROR("Global path search failed,%s", res.error_msg().c_str());
    }
  }
  isPose_ = !isPose_;
}
void AStar::pathVisualization(const nav_msgs::Path &path) {
  global_path_pub_.publish(path);
}
ErrorInfo AStar::searchPath(const Eigen::Vector2i &start_idx,
                            const Eigen::Vector2i &goal_idx,
                            nav_msgs::Path &path) {
  if (!map_->isInMap(start_idx)) {
    return ErrorInfo(
        ErrorCode::GP_START_INVALID_ERROR,
        "Global path can't be searched because the start is invalid");
  } else if (!map_->isInMap(goal_idx)) {
    return ErrorInfo(
        ErrorCode::GP_GOAL_INVALID_ERROR,
        "Global path can't be searched because the goal is invalid");
  } else {
    int start_adr = map_->toAdr(start_idx);
    int goal_adr = map_->toAdr(goal_idx);
    return searchPath(start_adr, goal_adr, path);
  }
}
ErrorInfo AStar::searchPath(const int &start_adr, const int &goal_adr,
                            nav_msgs::Path &path) {
  path.poses.clear();
  Node initNode{0, std::numeric_limits<int>::max(),
                std::numeric_limits<int>::max(),
                Node::SearchState::NOT_HANDLED};
  search_nodes_.assign(map_->getVoxelNum(), initNode);
  auto compare = [&](const int &adr1, const int &adr2) {
    return search_nodes_.at(adr1).g_score_ + search_nodes_.at(adr1).h_score_ >
           search_nodes_.at(adr2).g_score_ + search_nodes_.at(adr2).h_score_;
  };
  std::priority_queue<int, std::vector<int>, decltype(compare)> openlist(
      compare);
  search_nodes_.at(goal_adr).g_score_ = 0;
  openlist.push(start_adr);
  std::vector<int> neighbors_adr;
  int current_adr, move_cost, count = 0;
  while (!openlist.empty()) {
    current_adr = openlist.top();
    openlist.pop();
    search_nodes_.at(current_adr).search_state_ = Node::SearchState::CLOSED;
    if (current_adr == goal_adr) {
      break;
    }
    getNeighbors(current_adr, neighbors_adr);
    for (auto nbr : neighbors_adr) {
      if (map_->getCost(nbr) > 10 ||
          search_nodes_.at(nbr).search_state_ == Node::SearchState::CLOSED) {
        continue;
      }
      ErrorInfo res = getMoveCost(current_adr, nbr, move_cost);
      if (!res.IsOK()) {
        return res;
      }
      int new_g = search_nodes_.at(current_adr).g_score_ + move_cost +
                  map_->getCost(nbr);
      if (new_g < search_nodes_.at(nbr).g_score_) {
        search_nodes_.at(nbr).g_score_ = new_g;
        search_nodes_.at(nbr).parent_ = current_adr;
        if (search_nodes_.at(nbr).search_state_ ==
            Node::SearchState::NOT_HANDLED) {
          getDistance(nbr, goal_adr, search_nodes_.at(nbr).h_score_);
          openlist.push(nbr);
          search_nodes_.at(nbr).search_state_ = Node::SearchState::OPEN;
        }
      }
    }
    ++count;
  }
  if (current_adr != goal_adr) {
    // ROS_WARN("Valid global path not found %d", count);
    return ErrorInfo(ErrorCode::GP_PATH_SEARCH_ERROR,
                     "Valid global path not found");
  }
  int iter_adr = current_adr;
  geometry_msgs::PoseStamped iter_pos;
  Eigen::Vector2d pos;
  map_->idxToPos(map_->adrToIdx(iter_adr), pos);
  iter_pos.pose.position.x = pos(0);
  iter_pos.pose.position.y = pos(1);
  path.poses.emplace_back(iter_pos);
  while (iter_adr != start_adr) {
    iter_adr = search_nodes_.at(iter_adr).parent_;
    map_->idxToPos(map_->adrToIdx(iter_adr), pos);
    iter_pos.pose.position.x = pos(0);
    iter_pos.pose.position.y = pos(1);
    path.poses.emplace_back(iter_pos);
  }
  std::reverse(path.poses.begin(), path.poses.end());
  path.header.frame_id = "/map";
  path.header.stamp = ros::Time::now();
  // ROS_INFO("Global path length:%ld", path.poses.size());
  return ErrorInfo(ErrorCode::OK,
                   "Global path length: " + std::to_string(path.poses.size()));
}

void AStar::getNeighbors(const int &current_adr,
                         std::vector<int> &neighbors_adr) {
  neighbors_adr.clear();
  Eigen::Vector2i current_idx = map_->adrToIdx(current_adr);
  Eigen::Vector2i nbr_idx;
  for (int dx = -1; dx < 2; ++dx)
    for (int dy = -1; dy < 2; ++dy) {
      if (dx == 0 && dy == 0)
        continue;
      nbr_idx(0) = current_idx(0) + dx;
      nbr_idx(1) = current_idx(1) + dy;
      if (map_->isInMap(nbr_idx)) {
        neighbors_adr.emplace_back(map_->toAdr(nbr_idx));
      }
    }
}
ErrorInfo AStar::getMoveCost(const int &current_adr, const int &neighbor_adr,
                             int &move_cost) {
  int delta = abs(current_adr - neighbor_adr);
  if (delta == 1 || delta == map_->width()) {
    move_cost = 10;
  } else if (delta == map_->width() - 1 || delta == map_->width() + 1) {
    move_cost = 14;
  } else {
    return ErrorInfo(ErrorCode::GP_MOVE_COST_ERROR,
                     "Move cost can't be calculated cause current neighbor "
                     "index is not accessible");
  }
  return ErrorInfo(ErrorCode::OK);
}
void AStar::getDistance(const int &adr1, const int &adr2, int &distance) {
  Eigen::Vector2i idx1 = map_->adrToIdx(adr1);
  Eigen::Vector2i idx2 = map_->adrToIdx(adr2);
  int dx = std::abs(idx1(0) - idx2(0));
  int dy = std::abs(idx1(1) - idx2(1));

  // Euclidean distance
  //   distance = static_cast<int>(heuristic_factor_ * 10 * sqrt(dx * dx + dy *
  //   dy));

  // The best Heuristic
  distance = static_cast<int>(
      heuristic_factor_ *
      ((dx + dy) * 10 - 5.858f * std::min(dx, dy))); // sqrt(2)-2
}