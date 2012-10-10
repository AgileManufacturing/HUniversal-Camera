# camera # 
if(OpenCV_FOUND AND Boost_FOUND)
	camera_add_library(camera)
	include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
	target_link_libraries(camera ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${LIB_UNICAPCVBRIDGE_INCLUDE} )
else(OpenCV_FOUND AND Boost_FOUND)
	camera_cannot_build(camera "libraries/camera")
endif(OpenCV_FOUND AND Boost_FOUND)