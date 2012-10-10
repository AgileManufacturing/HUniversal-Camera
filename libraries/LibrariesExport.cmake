# Unfortunately, we need this so the ROS cmake files know where to look for our libraries :( 
# For every library we set global variables so our subjeprojects can find the includes and libraries they need. 

# TODO: Find a nicer way to take care of this. 

set(LIB_CAMERA_INCLUDE "${LCV_SOURCE_DIR}/libraries/camera/include")