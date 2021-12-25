cmake_minimum_required(VERSION 3.10)
project(stair_modeling_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -g")
set(CMAKE_BUILD_TYPE "Release")

find_package(
  catkin REQUIRED
  COMPONENTS dynamic_reconfigure
             cv_bridge
             image_transport
             roscpp
             message_filters
             sensor_msgs
             geometry_msgs
             std_msgs
             jsk_recognition_msgs
             tf
             stair_info_msg
             plane_msg)

# System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

catkin_package(
  # INCLUDE_DIRS include LIBRARIES stair_modeling_ros
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

include_directories(include/peac ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})

set(DSYEVH3_ROOT "3rdparty/dsyevh3")
include_directories(${DSYEVH3_ROOT})
set(DSYEVH3_SRC ${DSYEVH3_ROOT}/dsyevc3.c ${DSYEVH3_ROOT}/dsyevh3.c
                ${DSYEVH3_ROOT}/dsyevq3.c ${DSYEVH3_ROOT}/dsytrd3.c)
set_source_files_properties(${DSYEVH3_SRC} PROPERTIES LANGUAGE CXX)
add_library(lib3x3 SHARED ${DSYEVH3_SRC})
target_include_directories(lib3x3 PUBLIC ${DSYEVH3_ROOT})

add_executable(stair_modeling_node src/stair_modeling_node.cpp)
target_link_libraries(
  stair_modeling_node PRIVATE lib3x3 ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
                              ${catkin_LIBRARIES})
