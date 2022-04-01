cmake_minimum_required(VERSION 3.9.1)
########################################################################################################################
# Metadata
########################################################################################################################
# Read the package manifest.
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" package_xml_str)

# Extract project name.
if(NOT package_xml_str MATCHES "<name>([A-Za-z0-9_]+)</name>")
  message(FATAL_ERROR "Could not parse project name from package manifest (aborting)")
else()
  set(extracted_name ${CMAKE_MATCH_1})
endif()

# Extract project version.
if(NOT package_xml_str MATCHES "<version>([0-9]+.[0-9]+.[0-9]+)</version>")
  message(FATAL_ERROR "Could not parse project version from package manifest (aborting)")
else()
  set(extracted_version ${CMAKE_MATCH_1})
endif()

########################################################################################################################
# CMake project
########################################################################################################################
project(${extracted_name} VERSION ${extracted_version} LANGUAGES CXX)
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Wextra")
#===========================================================
# Dependencies
#===========================================================
# None

#-----------------------------
# Catkin packages
#-----------------------------
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  visualization_msgs
  geometry_msgs
  nav_msgs
  genmsg
  tf
)

find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    odometry_source_plugin
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    visualization_msgs
    geometry_msgs
    nav_msgs
    genmsg
    tf
    # ${PACKAGE_DEPENDENCIES}
  DEPENDS
    # gazebo_ros
    Boost
    GAZEBO
)

#===========================================================
# Build
#===========================================================

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

# #------------------------------------
# #  node_name
# #------------------------------------
add_library(
  odometry_source_plugin
        src/OdometrySource.cpp
)

add_dependencies(
  odometry_source_plugin
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  odometry_source_plugin
  ${catkin_LIBRARIES}
)

install(TARGETS odometry_source_plugin
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#===========================================================
# Install
#===========================================================

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)

install(
  DIRECTORY launch config
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
)

#===========================================================