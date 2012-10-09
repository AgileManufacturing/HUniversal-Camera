# unicap_cv_bridge # 
if(OpenCV_FOUND AND Boost_FOUND AND UNICAP_FOUND)
	camera_add_library(unicap_cv_bridge)
	include_directories(${UNICAP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
	target_link_libraries(unicap_cv_bridge ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${UNICAP_LIBRARIES})
else(OpenCV_FOUND AND Boost_FOUND AND UNICAP_FOUND)
	camera_cannot_build(unicap_cv_bridge "libraries/unicap_cv_bridge")
endif(OpenCV_FOUND AND Boost_FOUND AND UNICAP_FOUND)
