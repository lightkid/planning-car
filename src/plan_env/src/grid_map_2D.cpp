#include "plan_env/grid_map_2D.h"

// namespace cost_map {
CostMap::CostMap(ros::NodeHandle &nh, const std::string &topic_name) {
  nh_ = nh;
  map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>(
      topic_name, 2, &CostMap::mapCallback, this);
}
void CostMap::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map) {
  ROS_INFO("get map!");
  // get param
  double x_size, y_size, x_origin, y_origin;
  x_origin = map->info.origin.position.x;
  y_origin = map->info.origin.position.y;
  resolution_ = map->info.resolution;
  x_size = map->info.width; //图片的大小，不是实际尺寸
  y_size = map->info.height;
  frame_id_ = map->header.frame_id;

  resolution_inv_ = 1 / resolution_;
  map_origin_ = Eigen::Vector2d(x_origin, y_origin);
  map_size_ = Eigen::Vector2d(x_size, y_size);
  for (int i = 0; i < 2; ++i)
    map_voxel_num_(i) = std::floor(map_size_(i) / resolution_);
  map_min_boundary_ = map_origin_;
  map_max_boundary_ = map_origin_ + map_size_;
  std::cout << "origin: " << map_origin_.transpose() << std::endl;
  std::cout << "size: " << map_size_.transpose() << std::endl;
  std::cout << "minb: " << map_min_boundary_.transpose() << std::endl;
  std::cout << "maxb: " << map_max_boundary_.transpose() << std::endl;
  std::cout << "widthx, heighty: " << map_voxel_num_.transpose() << std::endl;
  int buffer_size = map_voxel_num_(0) * map_voxel_num_(1);
  cost_map_data_.resize(buffer_size, 0);
  std::cout << "map buffer length: " << map->data.size()
            << " costmap buffer length: " << buffer_size << std::endl;
  for (unsigned int w = 0; w < map_voxel_num_(0); ++w) {
    for (unsigned int h = 0; h < map_voxel_num_(1); ++h) {
      //应该维护多分辨率的供给hybridA*使用
      // auto x = static_cast<unsigned int>(
      //     (w + 0.5) * map_resolution /
      //     current_costmap_ptr_->info.resolution);
      // auto y = static_cast<unsigned int>(
      //     (h + 0.5) * map_resolution /
      //     current_costmap_ptr_->info.resolution);
      if (map->data[toAdr(w, h)])
        cost_map_data_[toAdr(w, h)] = 255;
      // if (current_costmap_ptr_
      //         ->data[y * current_costmap_ptr_->info.width + x]) {
      //   kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
      // }
    }
  }
  has_map_ = true;
  ROS_INFO("map init over");
}
void CostMap::posToIdx(const Eigen::Vector2d &pos, Eigen::Vector2i &idx) {
  idx(0) = floor((pos(0) - map_origin_(0)) * resolution_inv_);
  idx(1) = floor((pos(1) - map_origin_(1)) * resolution_inv_);
}
void CostMap::idxToPos(const Eigen::Vector2i &idx, Eigen::Vector2d &pos) {
  pos(0) = (idx(0) + 0.5) * resolution_ + map_origin_(0);
  pos(1) = (idx(1) + 0.5) * resolution_ + map_origin_(1);
}
int CostMap::toAdr(const int &x, const int &y) {
  return y * map_voxel_num_(0) + x;
}
int CostMap::toAdr(const Eigen::Vector2i &idx) {
  return idx(1) * map_voxel_num_(0) + idx(0);
}
Eigen::Vector2i CostMap::adrToIdx(const int &adr) {
  Eigen::Vector2i idx;
  idx(1) = adr / map_voxel_num_(0);
  idx(0) = adr % map_voxel_num_(0);
  return idx;
}

bool CostMap::isInMap(const Eigen::Vector2i &idx) {
  if (idx(0) < 0 || idx(1) < 0) {
    return false;
  }
  if (idx(0) > map_voxel_num_(0) - 1 || idx(1) > map_voxel_num_(1) - 1) {
    return false;
  }
  return true;
}
bool CostMap::isInMap(const Eigen::Vector2d &pos) {
  if (pos(0) < map_min_boundary_(0) + 1e-4 ||
      pos(1) < map_min_boundary_(1) + 1e-4) {
    // cout << "less than min range!" << endl;
    return false;
  }
  if (pos(0) > map_max_boundary_(0) - 1e-4 ||
      pos(1) > map_max_boundary_(1) - 1e-4) {
    return false;
  }
  return true;
}

uint8_t CostMap::getCost(const int &adr) { return cost_map_data_.at(adr); }
// } // namespace cost_map