##############################################################################
 # Macros
 # 
 # Created by Pascal Muller <pascalmuller@gmail.com>, 07-02-2012
 # 
 # 07-02-2012 Initial CMake support
 # 09-02-2012 Added functionality for building ROS packages as part of the whole. 
 # 01-10-2012 Dick vd Steen & Koen Braham - Changed buildsystem for the new camera achitecture.
 # 
 # == Description == 
 # This file adds macros we use in several CMakeLists. 
 # 
##############################################################################



##############################################################################
 # add_camera_library(<library_name>, [<path>])
 # 
 # == Description == 
 # This macro adds cpp sources and include files for a static library and sets 
 # up a build target for this library named <library_name>. 
 # 
 # This macro is made for our file structure, code is in <library_name>/src, 
 # headers in <library_name>/include
##############################################################################
macro(camera_add_library library_name)
	set(library_directory ${library_name})
	file(GLOB sources "src/*.cpp" "src/*.c")
	add_library(${library_name} STATIC ${sources})
	include_directories(BEFORE "include")
endmacro(camera_add_library)

##############################################################################
 # add_camera_executable(<executable_name>, [<path>])
 # 
 # == Description == 
 # This macro adds cpp sources and include files for a static library and sets 
 # up a build target for this library named <library_name>. 
 # 
 # This macro is made for our file structure, code is in <library_name>/src, 
 # headers in <library_name>/include
##############################################################################
macro(camera_add_executable executable_name)
	set(executable_directory ${executable_name})
	if(${ARGC} EQUAL 2)
		set(executable_directory ${ARGV1})
	endif(${ARGC} EQUAL 2)
	file(GLOB sources
		"${executable_directory}/src/*.cpp"
		"${executable_directory}/src/*.c"
	)
	add_executable(${executable_name} ${sources})
	
	include_directories(BEFORE "${executable_directory}/include")
endmacro(camera_add_executable)


##############################################################################
 # add_subdirectories(<sub_dirs>)
 # 
 # == Description == 
 # This macro executes add_subdirectory for all subdirectories in list. 
##############################################################################
macro(add_subdirectories sub_dirs)
	foreach(dir ${sub_dirs})
		#message("Added dir ${dir}")
		add_subdirectory(${dir})
	endforeach(dir)
endmacro(add_subdirectories)


##############################################################################
 # camera_rospack_init(<name> <gen_messages> <gen_services>)
 # 
 # == Description == 
 # This macro needs to be called in CMakeLists of our ROS packages. 
 # Parameters: Package name and two parameters that indicate wether ROSBUILD 
 # should generate messages and services. This is explained in the ROS tutorial. 
##############################################################################
macro(camera_rospack_init name gen_messages gen_services)
	set(ROSBUILD_DONT_REDEFINE_PROJECT TRUE)
	set(PROJECT_NAME "${name}")
	set(TEMPORARY_PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR})
	set(PROJECT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
	rosbuild_init()

	#set the default path for built executables to the "bin" directory
	set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
	#set the default path for built libraries to the "lib" directory
	set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

	if(${gen_messages})
		rosbuild_genmsg()
	endif(${gen_messages})
	if(${gen_services})
		rosbuild_gensrv()
	endif(${gen_services})
endmacro(camera_rospack_init)

##############################################################################
 # camera_rospack_end()
 # 
 # == Description == 
 # This macro MUST be called after lcv_rospack_init, after everything is done. 
##############################################################################
macro(camera_rospack_end)
	set(PROJECT_SOURCE_DIR ${TEMPORARY_PROJECT_SOURCE_DIR})
endmacro(camera_rospack_end)

##############################################################################
 # camera_cannot_build(<target_name> <friendly_name>)
 # 
 # == Description == 
 # This macro tells us that a package could not be built and adds a fake target 
 # to the makefile telling us the same when something does try to compile this 
 # target. target_name should be the name of the target, friendly_name is just 
 # for display purposes
##############################################################################
macro(camera_cannot_build target_name friendly_name)
	message(STATUS "${friendly_name} can't be build because libraries are missing. ") 
endmacro(camera_cannot_build)

##############################################################################
 # camera_check_ros()
 # 
 # == Description == 
 # This macro checks if rosbuild can be found and checks if camera_check_ros_path()
 # returns a valid return value. The value of ROS_OK is set accordingly. 
##############################################################################
macro(camera_check_ros)
	set(ROS_OK FALSE)
	if(NOT EXISTS "$ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake") #rosbuild not file available? Warn the user and omit the ROS targets
		message(WARNING "ROS NOT FOUND: Either ROS_ROOT is not set, or rosbuild.cmake could not be found. Check your setup. ROS packages cannot be build. ")
	else(NOT EXISTS "$ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake")
		# So we know we can include rosbuild, but is ROS_PACKAGE_PATH set correctly? If it isn't, rosbuild.cmake fails
		# so we should warn the user and omit the ROS targets. 
		camera_check_ros_path()
		if(ROS_PACKAGE_PATH_OK)
			set(ROS_OK TRUE)
		else(ROS_PACKAGE_PATH_OK)
			message(WARNING "The environment variable ROS_PACKAGE_PATH is not pointing to ${CMAKE_CURRENT_SOURCE_DIR} or a parent directory of it. ROS packages cannot be build. ")
		endif(ROS_PACKAGE_PATH_OK)
	endif(NOT EXISTS "$ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake")
endmacro(camera_check_ros)


##############################################################################
 # camera_check_ros_path()
 # 
 # == Description == 
 # This macro checks if the ROS_PACKAGE_PATH environment variable points to 
 # our repository and sets ROS_PACKAGE_PATH_OK accordingly. 
##############################################################################
macro(camera_check_ros_path)
	
	set(ROS_PACKAGE_PATH_OK FALSE)	
	string(REPLACE ":" ";" ROS_PACKPATHS_LIST "$ENV{ROS_PACKAGE_PATH}")
	list(LENGTH ROS_PACKPATHS_LIST listlen)
	# In the list should be at least two directories, one with all packages need for ROS itself and one for us. 
	# I'm not sure if I can check if the ROS directory is set correctly, as the location might change between versions of ROS. 
	if(NOT ${listlen} GREATER 1) 
		message(WARNING "ROS_PACKAGE_PATH needs to contain both the packages in the camera repository and the default paths. It should be configured in .bashrc after including ROS ssetup.bash. The rule should look like this: \n export ROS_PACKAGE_PATH=/home/username/gitrepos/low-cost-vision-2012/ROS:$ROS_PACKAGE_PATH\n")
		break()
	endif(NOT ${listlen} GREATER 1) 
	foreach(SINGLE_PACKDIR "${ROS_PACKPATHS_LIST}")
		set(ENVDIR "$ENV{ROS_PACKAGE_PATH}")
		string(REGEX REPLACE "^(.*)/$" "\\1" CENVDIR "${ENVDIR}" )
		string(LENGTH "${CENVDIR}" lengthres)
		if(${lengthres} GREATER 0)
			string(REGEX MATCH "^.*${CENVDIR}.*" RESULT "${ENVDIR} ${CMAKE_CURRENT_SOURCE_DIR}" )
		endif(${lengthres} GREATER 0)
		if(RESULT)
			set(ROS_PACKAGE_PATH_OK TRUE)
			break()		
		endif(RESULT)
	endforeach(SINGLE_PACKDIR ${ROS_PACKPATHS_LIST})
endmacro(camera_check_ros_path)



##############################################################################
# camera_add_doc(<doc_name>)
# 
# == Description == 
# This macro generates a documentation target
##############################################################################
macro(camera_add_doc doc_name)
       if(DOXYGEN_FOUND)
               set(DOCS_PATH "${CMAKE_SOURCE_DIR}")
               configure_file(${CMAKE_SOURCE_DIR}/cmake/Doxyfile.in ${CMAKE_BINARY_DIR}/doc/${doc_name}/Doxyfile @ONLY)
               add_custom_target("doc-${doc_name}"
                       ${DOXYGEN_EXECUTABLE} ${CMAKE_BINARY_DIR}/doc/${doc_name}/Doxyfile
                       WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/doc/${doc_name}
                       COMMENT "Generating documentation for target ${doc_name} with Doxygen" VERBATIM
               )
       endif(DOXYGEN_FOUND)
endmacro(camera_add_doc)