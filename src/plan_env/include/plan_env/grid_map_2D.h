#ifndef GRID_MAP_2D_H_
#define GRID_MAP_2D_H_

#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Eigen>
#include <memory>
#include <ros/ros.h>
#include <vector>

// namespace cost_map {
class CostMap {
  // pos 二维世界坐标
  // idx 二维栅格坐标
  // adr 栅格存储到一维数组的坐标
public:
  typedef std::shared_ptr<CostMap> Ptr;
  CostMap() = delete;
  CostMap(ros::NodeHandle &nh, const std::string &topic_name);
  /**
   * @brief 接收map_server发布的地图
   *
   * @param map
   */
  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &map);
  bool hasMap() { return has_map_; }
  void posToIdx(const Eigen::Vector2d &pos, Eigen::Vector2i &idx);
  void idxToPos(const Eigen::Vector2i &idx, Eigen::Vector2d &pos);
  int toAdr(const Eigen::Vector2i &idx);
  int toAdr(const int &x, const int &y);
  Eigen::Vector2i adrToIdx(const int &adr);

  bool isInMap(const Eigen::Vector2i &idx);
  bool isInMap(const Eigen::Vector2d &pos);

  uint8_t getCost(const int &adr);
  int width() { return map_voxel_num_(0); }
  int height() { return map_voxel_num_(1); }
  int getVoxelNum() { return map_voxel_num_(0) * map_voxel_num_(1); }

private:
  std::vector<uint8_t> cost_map_data_;

  Eigen::Vector2d map_origin_, map_size_;
  Eigen::Vector2d map_min_boundary_, map_max_boundary_;
  Eigen::Vector2i map_voxel_num_;
  double resolution_, resolution_inv_;
  std::string frame_id_;

  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  bool has_map_{false};
};

// } // namespace cost_map

#endif