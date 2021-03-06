##############################################################
# CMakeLists.txt for a ROS package within the low-cost-vision
# build system. 
# 
# Created by Pascal Muller <pascalmuller@gmail.com>, 09-02-2012
# 
# 09-02-2012 Initial version
# 
#############################################################

cmake_minimum_required(VERSION 2.4.6)
# Include all rosbuild macros
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Initialize ROS package to be in our build system. 
camera_rospack_init("cameraControlNode" false false)

# Add executables as documented by ROS
rosbuild_add_executable(CameraControlNode src/CameraControlNode.cpp)

# Add dependencies to our libraries and other ROS packages in the same directory as this one. 
add_dependencies(CameraControlNode CameraNode camera)

# Include our libraries, they are all exported in libraries/LibrariesExport.cmake
include_directories(${LIB_CAMERA_INCLUDE} ${NODE_CAMERA_INCLUDE} ${UNICAP_INCLUDE_DIRS})

# MUST be called after lcv_rospack_init to clean up after ROSBUILD!
camera_rospack_end()
