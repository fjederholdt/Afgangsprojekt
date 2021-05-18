/*
 * main.cpp
 *
 *  Created on: 4. mar. 2021
 *      Author: jeppe

#include <iostream>
#include <sstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

int main() {
	vector<Mat> chessimgs;
	/*InputArrayOfArrays objectpoints, imagePoints;
	InputOutputArray cameraMatrix, distCoeffs;
	OutputArrayOfArrays rvecs, tvecs;
	OutputArray stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors;
	TermCriteria criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, DBL_EPSILON);
	vector<Point3f> boardPoints;
	const string s = "/home/jeppe/opencv_build/opencv/samples/data/left";
	for (int i=1; i<15;i++)
	{
		if (i<10)
		{
			Mat img = imread(s+"0"+to_string(i)+".jpg");
			if (!img.empty())
			{
				chessimgs.push_back(img);
				cout << "found image: " << to_string(i)+".jpg" << endl;
			}
			else
				cout << "Didn't find image: " << to_string(i)+".jpg" << endl;
		}
		else{
			Mat img = imread(s+to_string(i)+".jpg");
			if (!img.empty())
			{
				chessimgs.push_back(img);
				cout << "found image: " << to_string(i)+".jpg" << endl;
			}
			else
				cout << "Didn't find image: " << to_string(i)+".jpg" << endl;
		}

	}
	Mat cameraMatrix;
	Mat distCoeffs;
	vector<Mat> R, T;
	vector<Point2f> corners;
	Size boardSize(9,6);
	Mat gray;
	vector<vector<Point3f>> objpoints;
	vector<vector<Point2f>> imgpoints;
	vector<Point3f> objp;
	for	(int i=0; i<boardSize.height; i++)
	{
		for (int j=0; j<boardSize.width; j++)
		{
			objp.push_back(Point3f(j,i,0));
		}
	}

	for (int i=0; i < chessimgs.size(); i++)
	{
		cvtColor(chessimgs.at(i), gray, COLOR_BGR2GRAY);
		bool found = findChessboardCorners(gray,boardSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE);
		if(found)
		{
			cornerSubPix(gray, corners, Size(5,5), Size(-1,-1), TermCriteria( TermCriteria::EPS+TermCriteria::COUNT, 30, 0.01));
			drawChessboardCorners(chessimgs.at(i), boardSize, Mat(corners), found);
			objpoints.push_back(objp);
			imgpoints.push_back(corners);
		}
		imshow("Image", chessimgs.at(i));
		waitKey(0);
	}
	destroyAllWindows();
	Size imageSize = Size(gray.rows, gray.cols);
	Mat view, rview, map1, map2;
	calibrateCamera(objpoints, imgpoints, imageSize, cameraMatrix, distCoeffs, R, T);

	cout << "cameraMatrix : " << cameraMatrix << endl;
	cout << "distCoeffs : " << distCoeffs << endl;

	Ptr<aruco::Dictionary> dictionary;

	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
	for (int i=0; i<chessimgs.size(); i++)
	{
		view = chessimgs.at(i);
		remap(view, rview, map1, map2, INTER_LINEAR);
		imshow("Remapped Image", rview);
		waitKey(0);
	}
	vector<Point2f> imgpoints2;
	vector<float> perViewErrors;
	perViewErrors.resize(objpoints.size());
	size_t totalPoints = 0;
	double totalErr = 0, err;
	for (int i=0; i<objpoints.size(); i++)
	{
		projectPoints(objpoints.at(i), R.at(i), T.at(i), cameraMatrix, distCoeffs, imgpoints2);
		err = cv::norm(imgpoints.at(i), imgpoints2, NORM_L2);
		size_t n = objpoints.at(i).size();
		perViewErrors.at(i) = (float) sqrt(err*err/n);
		totalErr += err*err;
		totalPoints += n;
	}
	cout << "Reprojection Error : " << sqrt(totalErr/totalPoints) << endl;
	//if (image.empty()) {
	//	printf("No image data \n");
	//	cout << s << endl;
	//	return -1;
	//}
		/*
	solvePnP(Mat(boardPoints), M)
	initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, image.size(), CV_32FC1, map1, map2);
	remap(image, )
	Mat temp = image.clone();
	namedWindow("Distorted Image", WINDOW_AUTOSIZE);
	imshow("Distored Image", image);

	namedWindow("Undistorted Image", WINDOW_AUTOSIZE);
	imshow("Undistored Image", image);
	waitKey(0);
	return 0;
}*/

