cmake_minimum_required(VERSION 2.8.3)
project(wyca_internship_test)

find_package(catkin REQUIRED
COMPONENTS roscpp
COMPONENTS rosbag)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
# INCLUDE_DIRS include
# LIBRAIRIES laser_scan_filter
# CATKIN_DEPENDS other_catkin_pkg
# DEPENDS system_lib
)


add_executable(laser_scan_filter src/laser_scan_filter.cpp)

target_link_libraries(laser_scan_filter ${catkin_LIBRARIES})
