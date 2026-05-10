cmake_minimum_required(VERSION 3.5)
project(minco_opt)

set(CMAKE_BUILD_TYPE "Release")
add_compile_options(-std=c++14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  traj_utils
  plan_env
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES minco_opt
 CATKIN_DEPENDS roscpp traj_utils plan_env
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(minco_opt
  src/minco_optimizer.cpp
)

target_link_libraries(minco_opt
  ${catkin_LIBRARIES}
)
