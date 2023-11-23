// #include "hybrid_a_star/rs_path.h"
#include "plan_env/grid_map_2D.h"

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>
Eigen::Vector3d pose_, goal_;
std::shared_ptr<CostMap> cost_map_ptr_;
ros::Publisher shape_vis_pub_;
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  double x1 = (*msg).pose.orientation.x;
  double y1 = (*msg).pose.orientation.y;
  double z1 = (*msg).pose.orientation.z;
  double w1 = (*msg).pose.orientation.w;
  double selfAngle1 = std::atan2(2 * (w1 * z1 + x1 * y1),
                                 1 - 2 * (y1 * y1 + z1 * z1)); // 偏航角
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
  std::cout << "pose: " << pose_.transpose() << std::endl;
  bool ans = cost_map_ptr_->isShapeInMap(pose_(0), pose_(1), pose_(2));
  std::cout << "is Shape In Map: " << ans << std::endl;
  bool res = cost_map_ptr_->isShapeCollision(pose_(0), pose_(1), pose_(2));
  std::cout << "is Shape Collision: " << res << std::endl;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "rs_test");
  ros::NodeHandle nh;
  cost_map_ptr_ = std::make_shared<CostMap>(nh, "/map");
  while (!cost_map_ptr_->hasMap() && ros::ok()) {
    ros::spinOnce(); // wait for map
  }
  cost_map_ptr_->setShape(2, 4);
  //   rs_path_ptr_ = std::make_shared<RSPath>(2);
  //   rs_curve_vis_pub_ = nh.advertise<nav_msgs::Path>("rs_vis", 10);
  shape_vis_pub_ = nh.advertise<visualization_msgs::Marker>("shape", 10);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 10, &goalCallback);

  ros::spin();
  return 0;
}