cmake_minimum_required(VERSION 3.10)
project(ic_gvins)

set(CMAKE_CXX_STANDARD 14)

# Optimization
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")

# Build GVINS library
include_directories(
        ${PROJECT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}/ic_gvins)
file(GLOB_RECURSE GVINS_SOURCE
        ic_gvins/*.cc
        ic_gvins/common/*.cc
        ic_gvins/fileio/*.cc
        ic_gvins/tracking/*.cc
        ic_gvins/preintegration/*.cc)
add_library(ic_gvins_core SHARED
        ${GVINS_SOURCE})

# OpenCV
find_package(OpenCV REQUIRED)
target_link_libraries(ic_gvins_core ${OpenCV_LIBS})

# glog
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Glog REQUIRED)
target_link_libraries(ic_gvins_core ${GLOG_LIBRARIES})

# Eigen3
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# yaml-cpp
find_package(yaml-cpp REQUIRED)
target_link_libraries(ic_gvins_core ${YAML_CPP_LIBRARIES})

# Ceres
find_package(Ceres REQUIRED)
target_link_libraries(ic_gvins_core ${CERES_LIBRARIES})

# boost
find_package(Boost REQUIRED COMPONENTS filesystem)
target_link_libraries(ic_gvins_core
        ${Boost_LIBRARIES})

# TBB
find_package(TBB REQUIRED)
target_link_libraries(ic_gvins_core
        TBB::tbb)

# abseil
set(ABSL_PROPAGATE_CXX_STD true)
add_subdirectory(thirdparty/abseil-cpp abseil-cpp)
target_link_libraries(ic_gvins_core
        absl::strings
        absl::str_format
        absl::time)

# ROS interface
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        message_generation
        nav_msgs
        roscpp
        sensor_msgs
        std_msgs
        tf2_ros
        visualization_msgs)

# NC-IC extension: the asynchronous relocation node receives optimized online
# snapshots and unbiased GNSS anchors through an explicit ROS boundary.
add_message_files(
        FILES
        RecoveryFrame.msg
        RecoveryEvent.msg)
generate_messages(
        DEPENDENCIES
        geometry_msgs
        std_msgs)
catkin_package(
        LIBRARIES
        CATKIN_DEPENDS geometry_msgs message_runtime nav_msgs roscpp sensor_msgs std_msgs tf2_ros visualization_msgs)
include_directories(
        ${catkin_INCLUDE_DIRS})

# GVINS ROS with standard messages
add_executable(${PROJECT_NAME}_ros
        ROS/fusion_ros.cc
        ROS/drawer_rviz.cc)

add_dependencies(${PROJECT_NAME}_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_ros
        ${catkin_LIBRARIES}
        ic_gvins_core)

# NC-IC extension: a loop-node-like asynchronous map correction process.
# It is separated from ic_gvins_ros so map correction never blocks online odom.
add_executable(${PROJECT_NAME}_reloc_node
        ROS/degraded_reloc_node.cc)
add_dependencies(${PROJECT_NAME}_reloc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_reloc_node
        ${catkin_LIBRARIES}
        ${CERES_LIBRARIES})
