cmake_minimum_required(VERSION 3.10)
project(local_grid)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -ffast-math")
set(CMAKE_BUILD_TYPE "Release")

find_package(
  catkin REQUIRED
  COMPONENTS dynamic_reconfigure
             cv_bridge
             roscpp
             message_filters
             sensor_msgs
             geometry_msgs
             std_msgs
             tf
             tf_conversions)

# System dependencies are found with CMake's conventions
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)

catkin_package(
  # INCLUDE_DIRS include LIBRARIES local_grid
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf
  # DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# link_directories(${PCL_LIBRARY_DIRS}) add_definitions(${PCL_DEFINITIONS})

add_executable(grid_node src/grid_node.cpp)
target_link_libraries(grid_node PRIVATE ${OpenCV_LIBRARIES} ${PCL_LIBRARIES}
                                        ${catkin_LIBRARIES})
