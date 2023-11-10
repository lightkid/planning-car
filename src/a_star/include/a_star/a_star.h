#ifndef A_STAR_HPP_
#define A_STAR_HPP_

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <memory>
#include <queue>

#include "a_star/error_code.h"
#include "plan_env/grid_map_2D.h"

struct Node {
  enum SearchState { NOT_HANDLED, OPEN, CLOSED };
  int h_score_;
  int g_score_;
  int parent_;
  SearchState search_state_;
};
class AStar {
public:
  typedef std::shared_ptr<AStar> Ptr;
  AStar() {}
  AStar(ros::NodeHandle &nh);
  ~AStar() {}

  void
  poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void pathVisualization(const nav_msgs::Path &path);

  // bool initMap(float heuristic_factor) {
  //   heuristic_factor_ = heuristic_factor;
  //   return true;
  // }
  ErrorInfo searchPath(const Eigen::Vector2i &start_idx,
                       const Eigen::Vector2i &goal_idx, nav_msgs::Path &path);
  ErrorInfo searchPath(const int &start_adr, const int &goal_adr,
                       nav_msgs::Path &path);
  void getNeighbors(const int &current_adr, std::vector<int> &neighbors_adr);
  // void getNeighbors(const Eigen::Vector3i &current_idx,
  //                   std::vector<Eigen::Vector3i> &neighbors_idx);
  ErrorInfo getMoveCost(const int &current_adr, const int &neighbor_adr,
                        int &move_cost);
  // ErrorInfo getMoveCost(const Eigen::Vector3i &current_idx,
  //                       const Eigen::Vector3i &neighbor_idx, int &move_cost);
  void getDistance(const int &adr1, const int &adr2, int &distance);
  // void getDistance(const Eigen::Vector3i &idx1, const Eigen::Vector3i &idx2,
  //                  int &distance);

private:
  ros::NodeHandle nh_;
  ros::Subscriber pose_sub_;
  ros::Subscriber goal_sub_;
  ros::Publisher global_path_pub_;
  Eigen::Vector2d pose_;
  Eigen::Vector2d goal_;

  bool isPose_{true}; // just for test

  std::vector<Node> search_nodes_;
  float heuristic_factor_{1.0};

  CostMap::Ptr map_;
};

#endif