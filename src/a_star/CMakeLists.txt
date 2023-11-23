cmake_minimum_required(VERSION 3.0.2)
project(a_star)

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

add_executable(a_star_demo
  src/a_star_test.cpp
  src/a_star.cpp
)
target_link_libraries(a_star_demo
  ${catkin_LIBRARIES}
)

