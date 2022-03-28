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
  tutorial_ros
)

catkin_package(
  INCLUDE_DIRS
    include
  # LIBRARIES
    ## Libraries to be built
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
  DEPENDS
    # External Libraries
    tutorial_ros
)

#===========================================================
# Build
#===========================================================

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#------------------------------------
#  localization
#------------------------------------
set(target_name localization)

## Declare a C++ executable
add_executable(${target_name}
  src/${target_name}_node.cpp
  src/${target_name}.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${target_name}
  ${catkin_LIBRARIES}
)

#------------------------------------
#  sensor_model
#------------------------------------
set(target_name sensor_model)

## Declare a C++ executable
add_executable(${target_name}
  src/${target_name}_node.cpp
  src/${target_name}.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${target_name}
  ${catkin_LIBRARIES}
)

#===========================================================
# Install
#===========================================================

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)

install(
  DIRECTORY config launch
  DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
)

#===========================================================