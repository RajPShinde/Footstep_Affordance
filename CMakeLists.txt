cmake_minimum_required(VERSION 3.0.2)
project(footstep_affordance)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
tf
tf_conversions
octomap_msgs
geometry_msgs
octomap_ros)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES footstep_affordance
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})

add_executable(footstepAffordance src/footstepAffordance.cpp src/features.cpp src/interface.cpp)
target_link_libraries(footstepAffordance ${catkin_LIBRARIES})
add_dependencies(footstepAffordance  ${catkin_EXPORTED_TARGETS})

