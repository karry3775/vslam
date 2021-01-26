cmake_minimum_required(VERSION 3.0.2)
project(visual_odometry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

# add opencv 
# set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
# message ("Found OpenCV: ${OpenCV_VERSION}" )


# add glog
find_package(glog 0.4.0 REQUIRED)
# add glfags 
find_package(gflags 0.4.0 REQUIRED)

# add boost
find_package(Boost COMPONENTS program_options python REQUIRED)

# add eigen
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# add pangolin
find_package(Pangolin REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES visual_odometry
  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
 ${Pangolin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/visual_odometry_node.cpp src/VisualOdometry.cpp src/PlottingUtils.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} 
                      ${OpenCV_LIBS} glog::glog gflags 
                      ${Boost_LIBRARIES}
                      ${Pangolin_LIBRARIES}
                      Eigen3::Eigen)


