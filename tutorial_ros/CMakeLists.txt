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
  genmsg
  tf
)

catkin_package(
  INCLUDE_DIRS
    include
)

#===========================================================
# Settings
#===========================================================
if(NOT DEFINED BUILD_SHARED_LIBS)
  set(BUILD_SHARED_LIBS ON)
endif()

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message(STATUS "${PROJECT_NAME}: Defaulting build type to RelWithDebInfo")
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

#===========================================================
# Build targets (for the ros_control hardware interface)
#===========================================================
set(target_name ${PROJECT_NAME}_tutorial_ros)

#-----------------------------
# Library target
#-----------------------------
set(
  src_files
    src/laser_text_display.cpp
)

add_library(${target_name} ${src_files})

target_include_directories(${target_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>"
  "${catkin_INCLUDE_DIRS}"
)

target_compile_features(${target_name} PRIVATE cxx_std_14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${target_name} PRIVATE -Wall -Wextra -Wpedantic)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  target_compile_options(${target_name} PRIVATE /W4)
endif()

#-----------------------------
# Executable target
#-----------------------------
set(
  src_files
    src/laser_text_display_node.cpp
)

add_executable(${target_name}_exe ${src_files})

target_include_directories(${target_name}_exe PRIVATE
  "${catkin_INCLUDE_DIRS}"
)

target_link_libraries(${target_name}_exe PRIVATE
  ${catkin_LIBRARIES}
  ${target_name}
)

target_compile_features(${target_name}_exe PRIVATE cxx_std_14)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(${target_name}_exe PRIVATE -Wall -Wextra -Wpedantic)
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
  target_compile_options(${target_name}_exe PRIVATE /W4)
endif()

set_target_properties(${target_name}_exe
  PROPERTIES
    OUTPUT_NAME "tutorial_ros"
)

#===========================================================
# Install
#===========================================================
install(
  TARGETS
    ${PROJECT_NAME}_tutorial_ros
    ${PROJECT_NAME}_tutorial_ros_exe
  ARCHIVE DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
  LIBRARY DESTINATION "${CATKIN_PACKAGE_LIB_DESTINATION}"
  RUNTIME DESTINATION "${CATKIN_PACKAGE_BIN_DESTINATION}"
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION "${CATKIN_PACKAGE_INCLUDE_DESTINATION}"
)

# install(
#   DIRECTORY config launch
#   DESTINATION "${CATKIN_PACKAGE_SHARE_DESTINATION}"
# )

# xx===============================================================xx
# ## Declare a cpp executable
# add_executable(laser_text_display_node src/laser_text_display_node.cpp)
# # add_executable(laser_graphical_display_node src/laser_graphical_display_node.cpp)
# # add_executable(detect_motion_node src/detect_motion_node.cpp)
# # add_executable(perform_clustering_node src/perform_clustering_node.cpp)
# # add_executable(rotation_done_node src/rotation_done_node.cpp)

# ## Specify libraries to link a library or executable target against
# target_link_libraries(laser_text_display_node ${catkin_LIBRARIES})
# # target_link_libraries(laser_graphical_display_node ${catkin_LIBRARIES})
# # target_link_libraries(detect_motion_node ${catkin_LIBRARIES})
# # target_link_libraries(perform_clustering_node ${catkin_LIBRARIES})
# # target_link_libraries(rotation_done_node ${catkin_LIBRARIES})
