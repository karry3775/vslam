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

# add dependencies for python
find_package(PythonLibs 2.7 REQUIRED)
# add boost
find_package(Boost COMPONENTS program_options python REQUIRED)

# adding gtkmm 
# find_package(PkConfig REQUIRED)
# pkg_check_modules(GTKMM gtkmm-2.4)
# find_package(GTKMM REQUIRED)
find_package (PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
pkg_check_modules(GTKMM gtkmm-3.0)
pkg_check_modules(GLIBMM glibmm-2.4)
pkg_check_modules(GIOMM giomm-2.4)
pkg_check_modules(GDKMM gdkmm-3.0)

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
 ${PYTHON_INCLUDE_DIRS}
 ${GTKMM_INCLUDE_DIRS}
 ${GTK3_INCLUDE_DIRS}
 ${GLIBMM_INCLUDE_DIRS}
 ${GIOMM_INCLUDE_DIRS}
 ${GDKMM_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/visual_odometry_node.cpp src/VisualOdometry.cpp src/PlottingUtils.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} 
                      ${OpenCV_LIBS} glog::glog gflags 
                      ${Boost_LIBRARIES} ${PYTHON_LIBRARIES}
                      ${GLIBMM_LIBRARIES}
                      ${GIOMM_LIBRARIES}
                      ${GTKMM_LIBRARIES}
                      ${GTK3_LIBRARIES}
                      ${GDKMM_LIBRARIES})


