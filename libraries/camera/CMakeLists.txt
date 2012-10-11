# camera # 
if(OpenCV_FOUND AND Boost_FOUND AND UNICAP_FOUND)
	camera_add_library(camera)
	include_directories(${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${UNICAP_INCLUDE_DIRS} )
	target_link_libraries(camera ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_FILESYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${UNICAP_LIBRARIES})
else(OpenCV_FOUND AND Boost_FOUND AND UNICAP_FOUND)
	camera_cannot_build(camera "libraries/unicap")
endif(OpenCV_FOUND AND Boost_FOUND AND UNICAP_FOUND)
