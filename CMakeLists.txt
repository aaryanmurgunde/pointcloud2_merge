cmake_minimum_required(VERSION 3.0.2)
project(pointcloud2_merge)

find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf2
    tf2_eigen
    tf2_ros
    message_filters
)

catkin_package(
    CATKIN_DEPENDS
    roscpp
    message_filters
)

find_package(PCL 1.7 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR NAMES yaml_cpp.h PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY NAMES YAML_CPP PATHS ${YAML_CPP_LIBRARY_DIRS})
link_directories(${YAML_CPP_LIBRARY_DIRS})

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

# Points Concat filter
add_executable(pointcloud2_merge
  src/pointcloud2_merge.cpp
)
add_dependencies(pointcloud2_merge ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud2_merge
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)