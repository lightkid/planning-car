#ifndef HYBRID_A_STAR_H_
#define HYBRID_A_STAR_H_

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

#include <Eigen/Eigen>
#include <memory>
#include <queue>

#include "hybrid_a_star/error_code.h"
#include "hybrid_a_star/rs_path.h"
#include "hybrid_a_star/type.h"
#include "plan_env/grid_map_2D.h"

struct StateNode {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  enum SearchState { NOT_HANDLED, OPEN, CLOSED };
  enum Direction { FORWARD, BACKWARD, NO };
  StateNode() = delete;
  explicit StateNode(const int &grid_adr) {
    search_state_ = NOT_HANDLED;
    adr_ = grid_adr;
    parent_ = nullptr;
  }
  int adr_; //在map中的编号，这个编号可以和
            // x y phi三维离散地图idx对应，为节省空间，不用idx
  SearchState search_state_{};
  Direction direction_{};
  double h_score_{};
  double g_score_{};
  double steering_{};
  Eigen::Vector3d pose_; // x y phi
  typedef std::shared_ptr<StateNode> Ptr;
  StateNode::Ptr parent_;
  VectorVec3d intermediate_states_; // vector中对齐的问题，命名比较长
};

class HybridAStar {
public:
  typedef std::shared_ptr<HybridAStar> Ptr;
  HybridAStar() {}
  HybridAStar(ros::NodeHandle &nh);
  ~HybridAStar() {}

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void pathVisualization(const nav_msgs::Path &path);

  ErrorInfo searchPath(const Eigen::Vector3d &start_pose,
                       const Eigen::Vector3d &goal_pose, nav_msgs::Path &path);
  double getMoveCost(const StateNode::Ptr &current_node_ptr,
                     const StateNode::Ptr &neighbor_node_ptr);
  void getNeighbors(const StateNode::Ptr &current_node,
                    std::vector<StateNode::Ptr> &neighbors_node);
  double getHeuristic(const StateNode::Ptr &current_node_ptr,
                      const StateNode::Ptr &terminal_node_ptr);
  bool AnalyticExpansions(const StateNode::Ptr &current_node_ptr,
                          const StateNode::Ptr &goal_node_ptr, double &length);
  inline void dynamicModel(const double &step, const double &steer, double &x,
                           double &y, double &phi);
  bool checkCollision(const double &x, const double &y, const double &theta);
  inline double mod2Pi(const double &phi);
  Eigen::Vector3i posToIdxHighRes(const Eigen::Vector3d &pos);
  void idxToPosHighRes(const Eigen::Vector3i &idx, Eigen::Vector3d &pos);
  int toAdrHighRes(const Eigen::Vector3i &idx);
  int toAdrHighRes(const Eigen::Vector3d &pos);
  Eigen::Vector3i adrToIdxHighRes(const int &adr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber goal_sub_;
  ros::Publisher global_path_pub_;
  Eigen::Vector3d pose_; // x y phi
  Eigen::Vector3d goal_; // x y phi

  bool isPose_{true}; // just for test

  CostMap::Ptr map_; //粗分辨率
  int grid_size_xy_{}; // map中一格有几个小栅格，细分辨率，需要跟大格配合
  int grid_size_phi_{}; //车yaw角的离散个数，单独使用
  double grid_resolution_xy_{};
  double grid_resolution_phi_{};
  std::map<int, StateNode::Ptr> search_tree_;

  // Car param
  double wheel_base_{};      //前后轮距
  double width_{};           //车宽
  double length_{};          //车长
  double steering_angle_{};  //最大打角
  double steering_radian_{}; //弧度
  Eigen::Matrix<double, 8, 1> car_corner_;
  int collision_radius_{};

  // Hybrid Astar param
  double segment_length_{};            //每步搜索的步长
  int steering_angle_discrete_num_{};  //打角离散数量
  double steering_radian_step_size_{}; //打角步长
  int segment_length_discrete_num_{};  //每步搜索积分离散点个数
  double move_step_size_{};            //积分前进步长

  double steering_penalty_{};        //打角惩罚->走直线
  double reversing_penalty_{};       //倒车惩罚->向前开
  double steering_change_penalty_{}; //打角变化惩罚->曲率变化
  double shot_distance_{};

  // rs curve
  std::shared_ptr<RSPath> rs_path_ptr_;

  // test
  ros::Publisher nbr_vis_pub_;
  ros::Publisher shape_vis_pub_;
  ros::Publisher select_point_pub_;
  nav_msgs::Path select_points_;
};

#endif