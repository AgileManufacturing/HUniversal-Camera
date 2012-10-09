#ifndef CAMERANODE_H
#define CAMERANODE_H

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <cv_bridge/CvBridge.h>
#include <CameraCalibration/RectifyImage.h>
#include <unicap_cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <sstream>
#include <std_srvs/Empty.h>

class CameraNode {
	public:
		CameraNode(int argc, char * argv[]);
		~CameraNode();
		void run();
		bool recalibrate(std_srvs::Empty &req, std_srvs::Empty &res);
	private:
		unicap_cv_bridge::unicap_cv_camera * cam;
		RectifyImage * rectifier;

		cv::Mat camFrame;
		cv::Mat rectifiedCamFrame;
		
		ros::NodeHandle nodeHandler;
		image_transport::ImageTransport imageTransporter;		
		image_transport::Publisher pub;

		int numberOfStableFrames;
		bool invokeCalibration;
};

#endif