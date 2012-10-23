# Camera Calibration
camera_add_executable(CameraCalibration)
# Include our libraries, they are all exported in Libraries/LibrariesExport.cmake
include_directories(${LIB_CAMERA_INCLUDE} ${UNICAP_INCLUDE_DIRS} ${LIB_UTILITIES_INCLUDE})
add_dependencies(CameraCalibration camera)
target_link_libraries(CameraCalibration camera)