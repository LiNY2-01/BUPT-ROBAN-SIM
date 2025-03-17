################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(humanoid_localization)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  octomap_msgs
  visualization_msgs
  cmake_modules
  tf
  message_filters
  octomap_ros
  pcl_ros
  pcl_conversions
)

find_package(Boost REQUIRED COMPONENTS random)
find_package(Eigen3 REQUIRED)

# required for OpenMP
find_package(OpenMP)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")

SET(LIBRARIES mapmodel motionmodel observationmodel raycastingmodel)

find_package(dynamicEDT3D)
if (${dynamicEDT3D_FOUND})
  include_directories(${DYNAMICEDT3D_INCLUDE_DIRS})
  link_directories(${DYNAMICEDT3D_LIBRARY_DIRS})
  link_libraries(${OCTOMAP_LIBRARIES} ${DYNAMICEDT3D_LIBRARIES})
  list(APPEND LIBRARIES endpointmodel)
else (${dynamicEDT3D_FOUND})
  MESSAGE(WARNING "dynamicEDT3D library (part of OctoMap >1.5) not found, skipping endpoint model")
  add_definitions(-DSKIP_ENDPOINT_MODEL)
endif (${dynamicEDT3D_FOUND})

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
  LIBRARIES ${LIBRARIES} humanoidlocalization
  CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    octomap_msgs
    visualization_msgs
    cmake_modules
    tf
    message_filters
    octomap_ros
    pcl_ros
    pcl_conversions
  DEPENDS Boost EIGEN3
)

################################################################################
# Build
################################################################################
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

if (${dynamicEDT3D_FOUND})
add_library(endpointmodel src/EndpointModel.cpp)
endif (${dynamicEDT3D_FOUND})

add_library(mapmodel src/MapModel.cpp)
add_library(motionmodel src/MotionModel.cpp)
add_library(observationmodel src/ObservationModel.cpp)
add_library(raycastingmodel src/RaycastingModel.cpp)
target_link_libraries(raycastingmodel observationmodel ${catkin_LIBRARIES})

add_library(humanoidlocalization src/HumanoidLocalization.cpp)
target_link_libraries(humanoidlocalization ${catkin_LIBRARIES} ${LIBRARIES})

add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node humanoidlocalization ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS ${LIBRARIES} humanoidlocalization
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS localization_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

################################################################################
# Test
################################################################################
