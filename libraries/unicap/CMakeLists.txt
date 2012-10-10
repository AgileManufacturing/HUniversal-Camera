# unicap # 
if(OpenCV_FOUND AND Boost_FOUND AND UNICAP_FOUND)
	camera_add_library(UNICAP_CV)
	include_directories(${Boost_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${UNICAP_INCLUDE_DIRS} ${UNICAP_INCLUDE_DIRS} )
	target_link_libraries(UNICAP_CV ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY} ${UNICAP_LIBRARIES})
else(OpenCV_FOUND AND Boost_FOUND AND UNICAP_FOUND)
	camera_cannot_build(UNICAP_CV "libraries/unicap")
endif(OpenCV_FOUND AND Boost_FOUND AND UNICAP_FOUND)
