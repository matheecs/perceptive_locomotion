cmake_minimum_required(VERSION 3.10)
project(peac_ros)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -march=native")
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS cv_bridge
             nodelet
             roscpp
             rostime
             sensor_msgs
             std_msgs
             tf
             message_filters
             dynamic_reconfigure
             image_transport
             cv_bridge
             message_generation
             message_runtime
             plane_msg)

catkin_package(
  # INCLUDE_DIRS include LIBRARIES peac
  CATKIN_DEPENDS
  cv_bridge
  nodelet
  roscpp
  rostime
  sensor_msgs
  std_msgs
  tf
  message_filters
  # DEPENDS system_lib
)

include_directories(include/peac ${EIGEN3_INCLUDE_DIR} ${catkin_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})

set(DSYEVH3_ROOT "3rdparty/dsyevh3")
include_directories(${DSYEVH3_ROOT})
set(DSYEVH3_SRC ${DSYEVH3_ROOT}/dsyevc3.c ${DSYEVH3_ROOT}/dsyevh3.c
                ${DSYEVH3_ROOT}/dsyevq3.c ${DSYEVH3_ROOT}/dsytrd3.c)
set_source_files_properties(${DSYEVH3_SRC} PROPERTIES LANGUAGE CXX)
add_library(lib3x3 SHARED ${DSYEVH3_SRC})
target_include_directories(lib3x3 PUBLIC ${DSYEVH3_ROOT})

add_executable(demo_peac src/demo_peac.cpp)
target_link_libraries(demo_peac PRIVATE lib3x3 ${OpenCV_LIBRARIES}
                                        ${catkin_LIBRARIES} ${PCL_LIBRARIES})
