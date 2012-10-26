/**
 * @file CameraCalibration.cpp
 * @brief Calibrate the camera
 * @date Created: 2012-10-22
 *
 * @author Koen Braham
 * @author Daan Veltman
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

#include <Camera/unicap_cv_bridge.h>
#include <Camera/RectifyImage.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cstdio>
#include <iostream>
#include <sstream>
#include <Utilities/Utilities.h>

int main(int argc, char** argv) {
	if(argc < 2) {
		std::cerr << "Usage for calibration application: " << std::endl << argv[0]
		        << " list - list of all unicap devices" << std::endl << argv[0]
		        << " capture device_number format_number [calibration_xml]" << std::endl << argv[0]
		        << " train image_directory calibration_xml" << std::endl << argv[0]
		        << " correct image_file calibration_xml" << std::endl;
		return -1;
	}

	std::string command = argv[1];
	if(command.compare("list") == 0) {
		unicap_cv_bridge::print_devices_inventory();
		return 0;
	} else if(command.compare("capture") == 0) {
		int device_number, format_number;
		if(Utilities::str2int(device_number, argv[2]) != 0) {
			std::cerr << "Device number is not a valid number." << std::endl;
			exit(2);
		}

		if(Utilities::str2int(format_number, argv[3]) != 0) {
			std::cerr << "Format number is not a valid number." << std::endl;
			exit(3);
		}

		unicap_cv_bridge::unicap_cv_camera cam(device_number, format_number);
		cv::Size camSize(cam.get_img_width(), cam.get_img_height());

		cv::Mat undistorted;
		Camera::RectifyImage rectifier;
		if(argc > 4)
			rectifier.initRectify(argv[4], camSize);

		cv::Mat frame(camSize, cam.get_img_format());

		cam.set_auto_white_balance(true);

		int counter = 0;
		double exposure = 0.015;
		double blue, red;
		char key = 0;
		bool tripmode = false;
		bool undistort = argc > 4;

		while(key != 'q') {
			cam.get_frame(&frame);

			if(undistort) {
				rectifier.rectify(frame, undistorted);
				imshow("undistorted", undistorted);
			} else {
				imshow("distorted", frame);
			}

			cam.get_white_balance(blue, red);

			key = cv::waitKey(10);
			std::stringstream ss;
			if(key == 'w') {
				exposure *= 1.125;
				cam.set_exposure(exposure);
			} else if(key == 's') {
				exposure /= 1.125;
				cam.set_exposure(exposure);
			} else if(key == 'a') {
				cam.set_auto_white_balance(true);
			} else if(key == 'z') {
				cam.set_auto_white_balance(false);
			} else if(key == 't') {
				tripmode = !tripmode;
				cam.set_trip_mode(tripmode);
			} else if(key == 'c') {
				if(argc > 4)
					undistort = !undistort;
			} else if(key == 'b') {
				ss.str("");
				ss << "Image" << counter++ << Utilities::timeNow() <<  ".jpg";
				if(undistort) {
					imwrite(ss.str().c_str(), undistorted);
				} else {
					imwrite(ss.str().c_str(), frame);
				}
				std::cout << "Image taken: " << ss.str().c_str() << std::endl;
				key = 0;
			}
			std::cout.flush();
		}
		return 0;
	} else if(command.compare("train") == 0) {
		if(argc < 3) {
			std::cerr << "Second argument has to be the image directory." << std::endl;
			return -1;
		} else if(argc < 4) {
			std::cerr << "Last argument has to be the newly generated file location." << std::endl;
			return -1;
		}

		/**
		 * @var boardSize
		 * boardSize is determined with the amount of inner corners for the width of the chessboard 
		 * (so 10 blocks is 9 inner corners) and the amount of inner corners for the height 
		 * of the chessboard (so 7 blocks is 6 inner corners)
		 */
		cv::Size boardSize(9, 6);
		Camera::RectifyImage rectifier;
		// TODO: Live calibration? Read X images of camera?
		if(rectifier.createXML(argv[2], boardSize, argv[3]) <= 0) {
			std::cerr << "training failed" << std::endl;
			return -1;
		}
		std::cout << "DONE training" << std::endl;
	} else if(command.compare("correct") == 0) {
		if(argc < 3) {
			std::cerr << "Second argument has to be the image file." << std::endl;
			return -1;
		} else if(argc < 4) {
			std::cerr << "Last argument has to be the calibration xml file." << std::endl;
			return -1;
		}

		Camera::RectifyImage rectifier;

		cv::Mat image = cv::imread(argv[3]);
		if(image.data == NULL) {
			std::cerr << "Unable to read image." << std::endl;
			exit(1);
		}
		if(!rectifier.initRectify(argv[2], cv::Size(image.cols, image.rows))) {
			std::cerr << "XML not found" << std::endl;
			return -1;
		}
		cv::imshow("Original", image);
		cvMoveWindow("Original", 0, 100);
		cv::Mat temp = image.clone();
		rectifier.rectify(image, temp);
		cv::imshow("Corrected", temp);
		cvMoveWindow("Corrected", image.cols, 100);

		while(1) {
			if( cvWaitKey (100) == 'q' ) break;
		}

		image.release();
		cv::destroyWindow("Corrected");
		cv::destroyWindow("Original");

		std::cout << "DONE correcting" << std::endl;
	}

	return 0;
}
