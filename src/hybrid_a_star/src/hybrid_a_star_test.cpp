#include "hybrid_a_star/hybrid_a_star.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "hybrid_a_star_demo");
  ros::NodeHandle nh;

  HybridAStar hybrid_astar_node(nh);
  ros::spin();
  return 0;
}