#include "a_star/a_star.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "a_star_demo");
  ros::NodeHandle nh;

  AStar astar_node(nh);
  ros::spin();
  return 0;
}