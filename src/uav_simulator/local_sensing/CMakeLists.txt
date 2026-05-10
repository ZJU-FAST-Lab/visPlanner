PROJECT(local_sensing_node)
CMAKE_MINIMUM_REQUIRED(VERSION 2.8.3)
SET(CMAKE_BUILD_TYPE Release) # Release, RelWithDebInfo
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )

option(ENABLE_CUDA "Enable CUDA accelerated depth renderer (set ON only when CUDA toolkit+GPU are ready)" OFF)
set(CUDA_ARCH "75" CACHE STRING "NVIDIA GPU compute capability, e.g. 61/75/86")

if(ENABLE_CUDA)
  message(STATUS "local_sensing_node: ENABLE_CUDA=ON, CUDA_ARCH=${CUDA_ARCH}")
  find_package(CUDA REQUIRED)

  # CUDA toolkit bundled with older ROS stacks may reject modern GCC by default.
  # Allow unsupported host compilers explicitly so catkin builds can proceed on newer distros.
  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 8.0)
    list(APPEND CUDA_NVCC_FLAGS --allow-unsupported-compiler)
    message(WARNING "GCC ${CMAKE_CXX_COMPILER_VERSION} detected; enabling NVCC flag --allow-unsupported-compiler")
  endif()

  list(APPEND CUDA_NVCC_FLAGS -O3 -use_fast_math)
  # NOTE: set CUDA_ARCH according to your GPU. Example: 61(TX2/GTX10), 75(Turing), 86(Ampere)
  list(APPEND CUDA_NVCC_FLAGS -gencode arch=compute_${CUDA_ARCH},code=sm_${CUDA_ARCH})

  SET(CUDA_PROPAGATE_HOST_FLAGS OFF)

  find_package(OpenCV REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(Boost REQUIRED COMPONENTS system filesystem)

  find_package(catkin REQUIRED COMPONENTS
      roscpp roslib cmake_modules cv_bridge image_transport pcl_ros sensor_msgs geometry_msgs nav_msgs quadrotor_msgs dynamic_reconfigure)
  generate_dynamic_reconfigure_options(
    cfg/local_sensing_node.cfg
  )
  catkin_package(
      DEPENDS OpenCV Eigen Boost
      CATKIN_DEPENDS roscpp roslib image_transport pcl_ros
  #INCLUDE_DIRS include
      LIBRARIES depth_render_cuda
  )

  include_directories(
    SYSTEM 
    #include 
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
  )

  CUDA_ADD_LIBRARY( depth_render_cuda
      src/depth_render.cu
  )

  add_executable(
    pcl_render_node
    src/pcl_render_node.cpp
  )
  target_link_libraries( pcl_render_node
    depth_render_cuda
    ${OpenCV_LIBS}
    ${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
  )
else(ENABLE_CUDA)
  message(WARNING "local_sensing_node: ENABLE_CUDA=OFF, using CPU pointcloud renderer.")
  find_package(Eigen3 REQUIRED)
  find_package(catkin REQUIRED COMPONENTS
      roscpp roslib cmake_modules pcl_ros sensor_msgs geometry_msgs nav_msgs quadrotor_msgs)
  
  catkin_package(
      DEPENDS Eigen
      CATKIN_DEPENDS roscpp roslib pcl_ros
  )

  include_directories(
    SYSTEM 
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}.
  )

  add_executable(
    pcl_render_node
    src/pointcloud_render_node.cpp
  )

  target_link_libraries( pcl_render_node
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
  )
endif(ENABLE_CUDA)
