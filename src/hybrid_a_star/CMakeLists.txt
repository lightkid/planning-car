cmake_minimum_required(VERSION 3.0.2)
project(hybrid_a_star)

set(CMAKE_BUILD_TYPE "Release")
# ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  plan_env
)
find_package(Eigen3 REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES a_star
 CATKIN_DEPENDS roscpp plan_env
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(hybrid_a_star_demo
  src/hybrid_a_star_test.cpp
  src/hybrid_a_star.cpp
  src/rs_path.cpp
  # src/rs_test.cpp
  # src/collision_check_test.cpp
)
target_link_libraries(hybrid_a_star_demo
  ${catkin_LIBRARIES}
)

