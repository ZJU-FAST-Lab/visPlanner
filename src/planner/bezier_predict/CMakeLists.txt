cmake_minimum_required(VERSION 2.8.3)
project(bezier_predict)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  cv_bridge
  traj_utils
  bspline_opt
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES bezier_predict
#  CATKIN_DEPENDS plan_env 
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
)

add_library( bezier_predict STATIC
    src/bezier_predict.cpp 
    src/predictor.cpp 
    )

target_link_libraries( bezier_predict
    ${catkin_LIBRARIES} 
    ooqpgensparse
    ooqpsparse 
    ooqpgondzio 
    ooqpbase blas ma27 gfortran f2c  
    )  
