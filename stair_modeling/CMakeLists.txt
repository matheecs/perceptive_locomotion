cmake_minimum_required(VERSION 3.10)
project(stair_modeling)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native")
set(CMAKE_BUILD_TYPE "Release")

find_package(
  catkin REQUIRED
  COMPONENTS dynamic_reconfigure
             plane_msg
             roscpp
             message_filters
             sensor_msgs
             geometry_msgs
             std_msgs
             jsk_recognition_msgs
             tf
             stair_info_msg)

# System dependencies are found with CMake's conventions
find_package(PCL 1.2 REQUIRED)

generate_dynamic_reconfigure_options(cfg/stair_modeling_params.cfg)

catkin_package(
  # INCLUDE_DIRS include LIBRARIES stair_modeling
  CATKIN_DEPENDS
  plane_msg
  roscpp
  sensor_msgs
  std_msgs
  tf
  stair_info_msg
  # DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(stair_modeling_node src/stair_modeling.cpp
                                   src/StairPerception.cpp)
target_link_libraries(stair_modeling_node ${PCL_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(stair_modeling_node ${PROJECT_NAME}_gencfg)
