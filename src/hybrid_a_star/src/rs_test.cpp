#include "geometry_msgs/PoseStamped.h"
#include "hybrid_a_star/rs_path.h"
#include "nav_msgs/Path.h"
#include <ros/ros.h>
bool isPose_;
std::shared_ptr<RSPath> rs_path_ptr_;
Eigen::Vector3d pose_, goal_;
ros::Publisher rs_curve_vis_pub_;
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
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
  } else {
    goal_(0) = msg->pose.position.x;
    goal_(1) = msg->pose.position.y;
    goal_(2) = selfAngle1;
    // generate RS curve
    double length;
    VectorVec3d rs_path_poses =
        rs_path_ptr_->GetRSPath(pose_, goal_, 0.2, length);
    // vis
    nav_msgs::Path rs_vis_path;
    rs_vis_path.header.frame_id = "map";
    rs_vis_path.header.stamp = ros::Time::now();
    for (auto pos : rs_path_poses) {
      geometry_msgs::PoseStamped pose;
      pose.pose.position.x = pos.x();
      pose.pose.position.y = pos.y();
      pose.pose.orientation.w = cos(pos.z() * 0.5);
      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = sin(pos.z() * 0.5);
      rs_vis_path.poses.push_back(pose);
    }
    rs_curve_vis_pub_.publish(rs_vis_path);
  }
  isPose_ = !isPose_;
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "rs_test");
  ros::NodeHandle nh;
  rs_path_ptr_ = std::make_shared<RSPath>(2);
  rs_curve_vis_pub_ = nh.advertise<nav_msgs::Path>("rs_vis", 10);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 10, &goalCallback);

  ros::spin();
  return 0;
}