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
Eigen::Vector2i CostMap::posToIdx(const Eigen::Vector2d &pos) {
  Eigen::Vector2i idx;
  idx(0) = floor((pos(0) - map_origin_(0)) * resolution_inv_);
  idx(1) = floor((pos(1) - map_origin_(1)) * resolution_inv_);
  return idx;
}
void CostMap::idxToPos(const Eigen::Vector2i &idx, Eigen::Vector2d &pos) {
  pos(0) = (idx(0) + 0.5) * resolution_ + map_origin_(0);
  pos(1) = (idx(1) + 0.5) * resolution_ + map_origin_(1);
}
Eigen::Vector2d CostMap::idxToPos(const Eigen::Vector2i &idx) {
  Eigen::Vector2d pos;
  pos(0) = (idx(0) + 0.5) * resolution_ + map_origin_(0);
  pos(1) = (idx(1) + 0.5) * resolution_ + map_origin_(1);
  return pos;
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

void CostMap::boundIndex(Eigen::Vector2i &idx) {
  Eigen::Vector2i id1;
  id1(0) = std::max(std::min(idx(0), map_voxel_num_(0) - 1), 0);
  id1(1) = std::max(std::min(idx(1), map_voxel_num_(1) - 1), 0);
  idx = id1;
}

bool CostMap::isOcc(const Eigen::Vector2i &idx) {
  return cost_map_data_.at(toAdr(idx)) > 10;
}

uint8_t CostMap::getCost(const int &adr) { return cost_map_data_.at(adr); }

void CostMap::setShape(const double &width, const double &length) {
  collision_radius_ = std::ceil(std::sqrt(width * width + length * length) *
                                0.5 * resolution_inv_);
  car_corner_.block<2, 1>(0, 0) = Eigen::Vector2d(length / 2, width / 2);
  car_corner_.block<2, 1>(2, 0) = Eigen::Vector2d(-length / 2, width / 2);
  car_corner_.block<2, 1>(4, 0) = Eigen::Vector2d(-length / 2, -width / 2);
  car_corner_.block<2, 1>(6, 0) = Eigen::Vector2d(length / 2, -width / 2);
  std::cout << "collision_radius: " << collision_radius_ << std::endl;
  //           << "car_corner0: " << car_corner_.block<2, 1>(0, 0) << std::endl
  //           << "car_corner1: " << car_corner_.block<2, 1>(2, 0) << std::endl
  //           << "car_corner2: " << car_corner_.block<2, 1>(4, 0) << std::endl
  //           << "car_corner3: " << car_corner_.block<2, 1>(6, 0) << std::endl;
}
bool CostMap::isShapeInMap(const double &x, const double &y,
                           const double &theta) {
  //四个角在矩形地图内，车就在内
  //车的四个角
  Eigen::Matrix2d R;
  R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
  Eigen::Vector2d corner;
  // Eigen::Matrix<double, 8, 1> transformed_corner;
  for (unsigned int i = 0; i < 4u; ++i) {
    corner = R * car_corner_.segment<2>(i * 2) + Eigen::Vector2d(x, y);
    // std::cout << "corner" << i << ": " << corner.transpose() << std::endl;
    if (!isInMap(corner)) {
      return false;
    }
  }
  return true;
}
bool poseInsideShape(std::vector<Eigen::Vector2d> &corners, int xp, int yp) {
  Eigen::Vector2d &A = corners[0];
  Eigen::Vector2d &B = corners[1];
  Eigen::Vector2d &C = corners[2];
  Eigen::Vector2d &D = corners[3];
  int a = (B.x() - A.x()) * (yp - A.y()) - (B.y() - A.y()) * (xp - A.x());
  int b = (C.x() - B.x()) * (yp - B.y()) - (C.y() - B.y()) * (xp - B.x());
  int c = (D.x() - C.x()) * (yp - C.y()) - (D.y() - C.y()) * (xp - C.x());
  int d = (A.x() - D.x()) * (yp - D.y()) - (A.y() - D.y()) * (xp - D.x());
  // std::cout << "abcd:" << a << " " << b << " " << c << " " << d << std::endl;
  if ((a > 0 && b > 0 && c > 0 && d > 0) ||
      (a < 0 && b < 0 && c < 0 && d < 0)) {
    return true;
  }
  return false;
}
bool CostMap::isShapeCollision(const double &x, const double &y,
                               const double &theta) {
  //确保车四个角都在地图内
  // if (!isShapeInMap(x, y, theta))
  //   return true;

  //对车中心周围的occ grid和车做矩形碰撞检测
  Eigen::Vector2i car_idx;
  posToIdx(Eigen::Vector2d(x, y), car_idx);
  // std::cout << "car_idx: " << car_idx.transpose() << std::endl;
  Eigen::Vector2i max_boundary =
      car_idx + Eigen::Vector2i(collision_radius_, collision_radius_);
  Eigen::Vector2i min_boundary =
      car_idx - Eigen::Vector2i(collision_radius_, collision_radius_);
  boundIndex(max_boundary);
  boundIndex(min_boundary);
  // std::cout << "max boundary: " << max_boundary.transpose() << std::endl;
  // std::cout << "min boundary: " << min_boundary.transpose() << std::endl;

  //车的四个角
  Eigen::Matrix2d R;
  R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
  std::vector<Eigen::Vector2d> car_corners;
  std::vector<Eigen::Vector2i> car_corners_idx;
  for (unsigned int i = 0; i < 4u; ++i) {
    car_corners.emplace_back(R * car_corner_.segment<2>(i * 2) +
                             Eigen::Vector2d(x, y));
    car_corners_idx.emplace_back(posToIdx(car_corners[i]));
    if (!isInMap(car_corners[i])) { //确保四个角在地图里
      return true;
    }
  }
  double r = resolution_ * 0.5;
  for (int dx = min_boundary(0); dx <= max_boundary(0); ++dx) {
    for (int dy = min_boundary(1); dy <= max_boundary(1); ++dy) {
      Eigen::Vector2i idx;
      idx(0) = dx;
      idx(1) = dy;
      if (isOcc(idx)) {
        // std::cout << "occ: " << idx.transpose() << std::endl;
        for (auto id : car_corners_idx) {
          if (id(0) == idx(0) && id(1) == idx(1)) {
            // shape的角点在grid中
            return true;
          }
        }
        // grid的4个点在shape中
        Eigen::Vector2d pos;
        idxToPos(idx, pos);
        bool ans = poseInsideShape(car_corners, pos(0) - r, pos(1) - r) ||
                   poseInsideShape(car_corners, pos(0) - r, pos(1) + r) ||
                   poseInsideShape(car_corners, pos(0) + r, pos(1) + r) ||
                   poseInsideShape(car_corners, pos(0) + r, pos(1) - r);
        // std::cout << "occ collision: " << ans << std::endl;
        if (ans) {
          return true;
        }
      }
    }
  }
  return false;
}
bool isShapeCollisionVis(const double &x, const double &y, const double &theta,
                         std::vector<Eigen::Vector2i> &collision_idxs) {}
// } // namespace cost_map