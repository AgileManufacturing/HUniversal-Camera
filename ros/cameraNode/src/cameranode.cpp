/**
 * @file Camera.h
 * @brief Camera node
 * @date Created: 2012-10-08
 * 
 * @author Arjan Groenewegen
 * 
 * @section LICENSE 
 * Copyright © 2012, HU University of Applied Sciences Utrecht. 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 * - Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
 * - Neither the name of the HU University of Applied Sciences Utrecht nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE HU UNIVERSITY OF APPLIED SCIENCES UTRECHT
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT 
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **/ 

#include "CameraNode/cameranode.h"

CameraNode::CameraNode(int argc, char * argv[]) 
:
it(nodeHandler)
{
	if (argc != 4){
		exit(1);
	}
	//setup the camera
	int device_number = atoi(argv[1]);
	int format_number = atoi(argv[2]);
	cam = new unicap_cv_bridge::unicap_cv_camera(device_number,format_number);
	cam->set_auto_white_balance(true);
	cam->set_exposure(0.015);
	camFrame = cv::Mat(cam->get_img_height(), cam->get_img_width(), cam->get_img_format());	

	//setup the camera lens distortion corrector
	rectifier = new Camera::RectifyImage();
	if(!rectifier->initRectify(argv[3], cv::Size(cam->get_img_width(), cam->get_img_height()))) {
		std::cout << "XML not found" << std::endl;
		exit(2);
	}	
	pub = it.advertise("camera/image", 1);
}

CameraNode::~CameraNode() {
	delete cam;
}

bool recalibrate(std_srvs::Empty &req, std_srvs::Empty &res) {
	return true;
}

void CameraNode::run() {
	
	ros::Rate loop_rate(5);
	while(ros::ok()){
		//read image 
		cam->get_frame(&camFrame);
		rectifier->rectify(camFrame, rectifiedCamFrame);

		//DEPRECATED		
		IplImage cv_output = rectifiedCamFrame;
		pub.publish(bridge.cvToImgMsg(&cv_output, "bgr8"));
		
		cv::waitKey(1000/30);
		
		ros::spinOnce();
	} 
}

int main(int argc, char* argv[]){
	ros::init(argc, argv, "cameraNode");

	CameraNode cn(argc, argv);
    cn.run();

    return 0;
}
