/**
 * @file Camera.cpp
 * @brief code for the camera class
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

#include "Camera.h"
#include <unicap_cv_bridge.h> 

Camera::Camera(int argc, char *argv[]) {
	//setup the camera
	int device_number = atoi(argv[1]);
	int format_number = atoi(argv[2]);
	cam = new unicap_cv_camera(device_number,format_number);
	cam->set_auto_white_balance(true);
	cam->set_exposure(0.015);
	camFrame = Mat(cam->get_img_height(), cam->get_img_width(), cam->get_img_format());				

	//setup the camera lens distortion corrector
	rectifier = new RectifyImage();
	if(!rectifier->initRectify(argv[3], cv::Size(cam->get_img_width(), cam->get_img_height()))) {
		cout << "XML not found" << endl;
		exit(2);
	}
}

void Camera::addPoints(const vector<Point2f>& imageCorners, const vector<Point3f>& objectCorners) {
    imagePoints.push_back(imageCorners);
    objectPoints.push_back(objectCorners);
}

double Camera::postProcessing(Size &imageSize) {
    vector<Mat> rvecs, tvecs;

    return calibrateCamera(	objectPoints,
		                    imagePoints,    // the image points
		                    imageSize,      // image size
		                    cameraMatrix,   // output camera matrix
		                    distCoeffs,     // output distortion matrix
		                    rvecs, tvecs,   // Rs, Ts
		                    0);             // set options
}

int RectifyImage::createXML(const char* imageDir, const Size &boardSize, const char* XMLName) {
    vector<Point2f> imageCorners;
    vector<Point3f> objectCorners;

    for(int i = 0; i<boardSize.height; i++) {
        for(int j = 0; j<boardSize.width; j++) {
            objectCorners.push_back(Point3f(i, j, 0.0f));
        }
    }

    if(!is_directory(imageDir)) {
        return -1;
    }

    Mat image;
    int successes = 0;
    for(directory_iterator iter = directory_iterator(imageDir); iter != directory_iterator(); iter++) {
        image = imread(iter->path().string().c_str(), 0);
        if(image.data) {
            bool found = findChessboardCorners(image, boardSize, imageCorners);

            if(found) {
                cornerSubPix(image, imageCorners, Size(4, 4), Size(-1, -1),
                	TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1));

                if(imageCorners.size() == (uint)boardSize.area()) {
                    addPoints(imageCorners, objectCorners);
                    successes++;
                }
            }
        }
    }

    if(!successes) {
        return -1;
    }

    Size imageSize(image.cols, image.rows);
    calibrate(imageSize);
    image.release();

    FileStorage fs(XMLName, FileStorage::WRITE);
    fs << "cameraMatrix" << cameraMatrix;
    fs << "distCoeffs" << distCoeffs;
    fs.release();
    return successes;
}

bool Camera::initRectify(const char* XMLName, const Size &imageSize) {
    if(!is_regular_file(XMLName)) {
        return false;
    }

    FileStorage fs(XMLName, FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), Mat(), imageSize, CV_32FC1, map1, map2);
    return true;
}

void Camera::rectify(const Mat &input, Mat &output) {
    remap(input, output, map1, map2, INTER_LINEAR);
}