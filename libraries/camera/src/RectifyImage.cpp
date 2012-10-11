
//******************************************************************************
//
//                 Low Cost Vision
//
//******************************************************************************
// Project:        CameraCalibration
// File:           RectifyImage.cpp
// Description:    The library to rectify images
// Author:         Glenn Meerstra & Zep Mouris
// Notes:          ...
//
// License:        GNU GPL v3
//
// This file is part of CameraCalibration.
//
// CameraCalibration is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// CameraCalibration is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with CameraCalibration.  If not, see <http://www.gnu.org/licenses/>.
//******************************************************************************
#include <Camera/RectifyImage.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <boost/filesystem.hpp>
#include <sstream>
#include <iostream>


namespace Camera
{
    using namespace cv;
    using namespace std;
    using namespace boost::filesystem;

    void RectifyImage::addPoints(const vector<Point2f>& imageCorners, const vector<Point3f>& objectCorners){
            imagePoints.push_back(imageCorners);
            objectPoints.push_back(objectCorners);
    }

    double RectifyImage::calibrate(Size &imageSize){
            vector<Mat> rvecs, tvecs;
            return calibrateCamera(objectPoints,
                            imagePoints,    // the image points
                            imageSize,      // image size
                            cameraMatrix,   // output camera matrix
                            distCoeffs,     // output distortion matrix
                            rvecs, tvecs,   // Rs, Ts
                            0);             // set options
    }

    int RectifyImage::createXML(const char* imageDir, const Size &boardSize, const char* XMLName){
            vector<Point2f> imageCorners;
            vector<Point3f> objectCorners;

            for(int i = 0; i < boardSize.height; i++) {
                    for(int j = 0; j < boardSize.width; j++) {
                            objectCorners.push_back(Point3f(i, j, 0.0f));
                    }
            }

            if(!is_directory(imageDir)){
                    return -1;
            }

            Mat image;
            int successes = 0;
            for (directory_iterator iter = directory_iterator(imageDir); iter != directory_iterator(); iter++) {
                    image = imread(iter->path().string().c_str(), 0);
                    if(image.data){
                            bool found = findChessboardCorners(image, boardSize, imageCorners);

                            if(found) {
                                    cornerSubPix(image, imageCorners, Size(4, 4), Size(-1, -1),
                                    TermCriteria(TermCriteria::MAX_ITER + TermCriteria::EPS, 30, 0.1));

                                    if (imageCorners.size() == (uint)boardSize.area()) {
                                            addPoints(imageCorners, objectCorners);
                                            successes++;
                                    }
                            }
                    }
            }

            if(!successes){
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

    bool RectifyImage::initRectify(const char* XMLName, const Size &imageSize){
            if(!is_regular_file(XMLName)){
                    return false;
            }

            FileStorage fs(XMLName, FileStorage::READ);
            fs["cameraMatrix"] >> cameraMatrix;
            fs["distCoeffs"] >> distCoeffs;
            initUndistortRectifyMap( cameraMatrix, distCoeffs, Mat(), Mat(), imageSize, CV_32FC1, map1, map2);
            return true;
    }

    void RectifyImage::rectify(const Mat &input, Mat &output){
            remap(input, output, map1, map2, INTER_LINEAR);
    }
}