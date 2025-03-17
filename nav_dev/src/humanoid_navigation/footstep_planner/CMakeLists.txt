################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(footstep_planner)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  actionlib
  rosconsole
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  humanoid_nav_msgs
  tf
  angles
  gridmap_2d
  map_server
)

find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
include_directories(${SBPL_INCLUDE_DIRS})
link_directories(${SBPL_LIBRARY_DIRS})

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    rospy
    actionlib
    rosconsole
    sensor_msgs
    geometry_msgs
    nav_msgs
    visualization_msgs
    humanoid_nav_msgs
    tf
    angles
    gridmap_2d
    map_server
  DEPENDS OpenCV SBPL
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
)

set(FOOTSTEP_PLANNER_FILES 
  src/FootstepPlanner.cpp
	src/FootstepNavigation.cpp
  src/FootstepPlannerNode.cpp
  src/FootstepPlannerEnvironment.cpp 
  src/Footstep.cpp
  src/PlanningState.cpp
  src/Heuristic.cpp 
  src/helper.cpp
  src/PathCostHeuristic.cpp
  src/PlanningStateChangeQuery.cpp
  src/State.cpp
)

add_library(${PROJECT_NAME} ${FOOTSTEP_PLANNER_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${SBPL_LIBRARIES})

add_executable(footstep_planner_node src/footstep_planner.cpp)
add_dependencies(footstep_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(footstep_planner_node ${PROJECT_NAME} ${SBPL_LIBRARIES})

add_executable(footstep_planner_walls src/footstep_planner_walls.cpp)
add_dependencies(footstep_planner_walls ${catkin_EXPORTED_TARGETS})
target_link_libraries(footstep_planner_walls ${PROJECT_NAME} ${OpenCV_LIBRARIES} ${SBPL_LIBRARIES})

add_executable(footstep_navigation_node src/footstep_navigation.cpp)
add_dependencies(footstep_navigation_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(footstep_navigation_node ${PROJECT_NAME} ${SBPL_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS footstep_planner_node footstep_planner_walls footstep_navigation_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS
  scripts/corrected_initialpose.py
  scripts/plan_footsteps.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch maps 
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
