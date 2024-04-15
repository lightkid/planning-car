#include "plan_env/grid_map_2D.h"
#define BACKWARD_HAS_DW 1
#include "backward.hpp"
namespace backward {
backward::SignalHandling sh;
}
// namespace cost_map {
CostMap::CostMap(ros::NodeHandle &nh, const std::string &topic_name) {
  nh_ = nh;
  esdf_pub_ = nh_.advertise<sensor_msgs::Image>("/esdf", 10);
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
  unsigned int buffer_size = map_voxel_num_(0) * map_voxel_num_(1);
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
  auto start_time = ros::Time::now();
  updateESDF();
  auto end_time = ros::Time::now();
  ROS_INFO("update esdf cost: %f ms", (end_time - start_time).toSec() * 1000);
  ESDFVis();
}
bool CostMap::updateESDF() {
  // TODO: update esdf
  // ref: FIESTA: Fast Incremental Euclidean Distance Fields for Online Motion
  // Planning of Aerial Robots Alg.1
  unsigned int buffer_size = map_voxel_num_(0) * map_voxel_num_(1);
  esdf_map_data_.resize(buffer_size);
  std::queue<EsdfGridData::Ptr> update_queue;
  for (int adr = 0; adr < buffer_size; ++adr) {
    esdf_map_data_[adr] = std::make_shared<EsdfGridData>(adr);
    if (cost_map_data_[adr] == 255) {
      esdf_map_data_[adr]->setOcc();
      update_queue.push(esdf_map_data_[adr]);
    }
  }
  auto Dist = [this](Adr adr1, Adr adr2) {
    Eigen::Vector2i idx1 = adrToIdx(adr1);
    Eigen::Vector2i idx2 = adrToIdx(adr2);
    int dx = std::abs(idx1(0) - idx2(0));
    int dy = std::abs(idx1(1) - idx2(1));
    // Euclidean distance
    return sqrt(double(dx * dx + dy * dy));
  };
  int count = 1;
  while (!update_queue.empty()) {
    auto cur = update_queue.front();
    update_queue.pop();
    getNeighbors(cur);
    double max_dist = 0.0;
    for (auto nbr_adr : cur->nbrs) {
      auto &nbr = esdf_map_data_[nbr_adr];
      double dist = Dist(cur->coc, nbr_adr);
      max_dist = std::max(max_dist, dist);
      if (dist < nbr->dist) { // Absolutely free
        // 将nbr从nbr.coc的dll中删除
        if (nbr->coc != IdealPointAdr) {
          auto &oldcoc = esdf_map_data_[nbr->coc];
          oldcoc->dll.erase(nbr->adr);
        }
        // 将nbr加入cur的coc的dll
        auto &newcoc = esdf_map_data_[cur->coc];
        nbr->setCoc(newcoc->adr, dist);
        newcoc->dll.insert(nbr->adr);
        //加入update queue
        update_queue.push(nbr);
      }
    }
    // std::cout << std::endl;
    // if (count == 145) {
    //   break;
    // }
    // ++count;
  }
  // TODO:使用opencv显示灰度图
  // cv::Mat res(cv::Size(map_voxel_num_(1), map_voxel_num_(0)), CV_32FC1,
  //             cv::Scalar(0));
  // std::cout << "ok2" << std::endl;
  // for (unsigned int w = 0; w < map_voxel_num_(0); ++w) {
  //   for (unsigned int h = 0; h < map_voxel_num_(1); ++h) {
  //     if (esdf_map_data_[toAdr(w, h)]->dist > 1000.0) {
  //       std::cout << "-"
  //                 << "\t";
  //     } else {
  //       std::cout << std::setprecision(2) << esdf_map_data_[toAdr(w,
  //       h)]->dist
  //                 << "\t";
  //     }
  //     res.at<float>(w, h) =
  //         (float)(esdf_map_data_[toAdr(w, h)]->dist); // TODO： 不要用at
  //   }
  //   std::cout << std::endl;
  // }
  // cv_bridge::CvImage out_msg;
  // out_msg.header.stamp = ros::Time::now();
  // out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  // out_msg.image = res.clone();
  // ros::Publisher pureb_depth = nh_.advertise<sensor_msgs::Image>("/esdf",
  // 10); for (int i = 0; i < 100; ++i) {
  //   pureb_depth.publish(out_msg.toImageMsg());
  //   ros::Duration(0.1).sleep();
  // }

  ROS_INFO("esdf map update over");
  return true;
}
void CostMap::ESDFVis() {
  // TODO:使用opencv显示灰度图
  cv::Mat res(cv::Size(map_voxel_num_(1), map_voxel_num_(0)), CV_32FC1,
              cv::Scalar(0));
  for (unsigned int w = 0; w < map_voxel_num_(0); ++w) {
    for (unsigned int h = 0; h < map_voxel_num_(1); ++h) {
      // if (esdf_map_data_[toAdr(w, h)]->dist > 1000.0) {
      //   std::cout << "-"
      //             << "\t";
      // } else {
      //   std::cout << std::setprecision(2) << esdf_map_data_[toAdr(w,
      //   h)]->dist
      //             << "\t";
      // }
      res.at<float>(w, h) = (float)(esdf_map_data_[toAdr(w, h)]->dist);
    }
    // std::cout << std::endl;
  }
  cv_bridge::CvImage out_msg;
  out_msg.header.stamp = ros::Time::now();
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  out_msg.image = res.clone();
  for (int i = 0; i < 100; ++i) {
    esdf_pub_.publish(out_msg.toImageMsg());
    ros::Duration(0.1).sleep();
  }
}
void CostMap::getNeighbors(EsdfGridData::Ptr &node) {
  node->nbrs.clear();
  Idx current_idx = adrToIdx(node->adr);
  Idx nbr_idx;
  // 8-connectivity
  // for (int dx = -1; dx < 2; ++dx)
  //   for (int dy = -1; dy < 2; ++dy) {
  //     if (dx == 0 && dy == 0)
  //       continue;
  //     nbr_idx(0) = current_idx(0) + dx;
  //     nbr_idx(1) = current_idx(1) + dy;
  //     if (isInMap(nbr_idx)) {
  //       node->nbrs.emplace_back(toAdr(nbr_idx));
  //     }
  //   }
  // 4-connectivity
  std::vector<Idx> dirs = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};
  for (Idx &dir : dirs) {
    nbr_idx = current_idx + dir;
    if (isInMap(nbr_idx)) {
      node->nbrs.emplace_back(toAdr(nbr_idx));
    }
  }
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
  //           << "car_corner0: " << car_corner_.block<2, 1>(0, 0) <<
  //           std::endl
  //           << "car_corner1: " << car_corner_.block<2, 1>(2, 0) <<
  //           std::endl
  //           << "car_corner2: " << car_corner_.block<2, 1>(4, 0) <<
  //           std::endl
  //           << "car_corner3: " << car_corner_.block<2, 1>(6, 0) <<
  //           std::endl;
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
  double a = (B.x() - A.x()) * (yp - A.y()) - (B.y() - A.y()) * (xp - A.x());
  double b = (C.x() - B.x()) * (yp - B.y()) - (C.y() - B.y()) * (xp - B.x());
  double c = (D.x() - C.x()) * (yp - C.y()) - (D.y() - C.y()) * (xp - C.x());
  double d = (A.x() - D.x()) * (yp - D.y()) - (A.y() - D.y()) * (xp - D.x());
  // std::cout << "abcd:" << a << " " << b << " " << c << " " << d <<
  // std::endl;
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