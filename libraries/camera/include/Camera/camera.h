/**
 * @file Camera.h
 * @brief Header for the camera class
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

#ifndef CAMERA_H
#define CAMERA_H

#include <unicap_cv_bridge.h>

class Camera {
	public: 
		Camera(); //constructor

 		double postProcessing();
 		void sendData();

	 	/**
		* Creates a matrix from all the images located in imageDir and stores it in the XMl file
		*
		* @param imageDir the directory which contains the image to rectify
		* @param boardSize amount of squares horizontally -1, amount of squares vertically -1
		* @param XMLName the name of the xml file were the matrix for rectification is written to
		* @return the amount of images which are successfully processed
		*/
		int createXML(const char* imageDir, const cv::Size &boardSize, const char* XMLName);
		/**
		* this function loads a matrix to rectify
		*
		* @param XMLName the name of the XML
		* @param imageSize the size of the image
		* @return <i>false</i> if XMLName is not available
		*/
		bool initRectify(const char* XMLName, const cv::Size &imageSize);
		/**
		* Returns the given image rectified
		*
		* @param input The image that needs to be rectified
		* @param output The rectified image
		*/
		void rectify(const cv::Mat &input, cv::Mat &output);

 	private:
 		unicap_cv_bridge::unicap_cv_camera * cam;
		std::vector<std::vector<cv::Point3f> > objectPoints;
		std::vector<std::vector<cv::Point2f> > imagePoints;
		cv::Mat distCoeffs;
		cv::Mat cameraMatrix;
		cv::Mat map1;
		cv::Mat map2;


		void addPoints(const std::vector<cv::Point2f>& imageCorners, const std::vector<cv::Point3f>& objectCorners);
		double calibrate(cv::Size &imageSize);
};

 #endif
