#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <ctime>
#include <cstdio>
#include <vector>
#include <time.h>
#include <math.h>
#include <thread>


#include <opencv2/imgproc.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
#include <pcl/features/normal_3d.h>


#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde.h>

#include "FileStorageClass.h"
#include "DHParams.h"
//#include "TCPServer.h"

using namespace cv;
using namespace std;

const float chessSquareDim = 0.02f;
const float arucoSquareDim = 0.015f;
const Size chessboardDim = Size(9, 14);

double findArucoMarkers(vector<Mat>& images, Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvectors, vector<Mat>& tvectors)
{
	try
	{
	Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100);
	Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(chessboardDim.height, chessboardDim.width, chessSquareDim, arucoSquareDim, dictionary);
	vector<int> markerIds;
	vector<vector<int>> markerIdsVec;
	vector<vector<Point2f>> markerCorners;
	vector<vector<Point2f>> markerCornersVec;
	vector<Mat> rvecs, tvecs;
	Size imageSize = Size(images.at(0).rows, images.at(0).cols);
	Mat boardImage;
	board->draw(Size(600, 400), boardImage, 1, 1);
	//imshow("Board Image", boardImage);
	//waitKey(0);

	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
	{
		Mat inputImage = *iter;
		aruco::detectMarkers(inputImage, board->dictionary, markerCorners, markerIds);
		if (markerIds.size() > 0)
		{
			vector<Point2f> charucoCorners;
			vector<int> charucoIds;
			aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImage, board, charucoCorners, charucoIds);
			markerIdsVec.push_back(charucoIds);
			markerCornersVec.push_back(charucoCorners);
			if(charucoIds.size() > 0 && !cameraMatrix.empty())
			{
				aruco::drawDetectedCornersCharuco(inputImage, charucoCorners, charucoIds);
				Vec3d rvec, tvec;
				Mat rMat, tMat;
				bool valid = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
				if (valid)
				{
					aruco::drawAxis(inputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
					//imshow("charuco", inputImage);
					Rodrigues(rvec, rMat);
					Rodrigues(tvec, tMat);
					rvectors.push_back(rMat);
					tvectors.push_back(tMat);
					//cout << "rvec: " << endl << rvec << endl << "tvec: " << endl << tvec << endl;
					//waitKey(0);
				}
			}
		}
	}
	if (cameraMatrix.empty())
	{
		return aruco::calibrateCameraCharuco(markerCornersVec, markerIdsVec, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
	}
	else
		return 0;
	}
	catch( cv::Exception& e )
	{
	    const char* err_msg = e.what();
	    std::cout << "exception caught: " << err_msg << std::endl;
	}
	return 0;
}

vector<Mat> getImages(const string s){
	vector<Mat> chessimgs;
	for (int i=0; i<54;i++)
	{
		if (i<10)
		{
			Mat img = imread(s+"0"+to_string(i)+".png");
			if (!img.empty())
			{
				chessimgs.push_back(img);
				cout << "found image: 000" << to_string(i)+".png" << endl;
			}
			else
				cout << "Didn't find image: 000" << to_string(i)+".png" << endl;
		}
		else{
			Mat img = imread(s+to_string(i)+".png");
			if (!img.empty())
			{
				chessimgs.push_back(img);
				cout << "found image: 00" << to_string(i)+".png" << endl;
			}
			else
				cout << "Didn't find image: 00" << to_string(i)+".png" << endl;
		}
	}
	return chessimgs;
}


void remapping(vector<Mat>& images, const Mat& cameraMatrix, const Mat& distCoeffs,const Size imageSize, Mat& map1, Mat& map2){
	Mat view, rview;
	initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
	for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
	{
		view = *iter;
		remap(view, rview, map1, map2, INTER_LINEAR);
		//imshow("Remapped Image", rview);
		//waitKey(0);
	}
}

pcl::visualization::PCLVisualizer::Ptr viz (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
	pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0, 0, 255);
	viewer->addPointCloud<pcl::PointXYZ> (cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem (1);
	//viewer->initCameraParameters ();
	return (viewer);
}

using namespace std::chrono_literals;

int main() {
	//TCPServer server;
	//server.sendSocket("Hello");

	ur_rtde::RTDEReceiveInterface rtde_receive("192.168.20.128");
	vector<double> joint_positions = rtde_receive.getActualQ();
	for (int i = 0; i < joint_positions.size(); i++)
	{
		cout << joint_positions.at(i) << " ";
	}

	ur_rtde::RTDEControlInterface rtde_control("192.168.20.128");
	rtde_control.moveL({-0.143, -0.435, 0.20, -0.001, 3.12, 0.04}, 0.5, 0.2);

	std::this_thread::sleep_for(1s);

	ur_rtde::RTDEReceiveInterface rtde_receive2("192.168.20.128");
	vector<double> joint_positions2 = rtde_receive2.getActualQ();
	for (int i = 0; i < joint_positions2.size(); i++)
	{
		cout << joint_positions2.at(i) << " ";
	}

	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointXYZ robot;
	pcl::PointXYZ camera;
	pcl::PointXYZ handeye;

	time_t rawtime; time(&rawtime);
	FileStorageClass fsCamera("cameraData.yml", asctime(localtime(&rawtime)));
	FileStorageClass fsRobot("robotData.yml", asctime(localtime(&rawtime)));
	FileStorageClass fsHandEye("handEyeData.yml", asctime(localtime(&rawtime)));
	const string s = "/home/jeppe/Hentet/charuco/calib/00";
	vector<Mat> images = getImages(s);
	const Size imageSize = Size(images.at(0).rows, images.at(0).cols);
	Mat cameraMatrix, distCoeffs, map1, map2;
	vector<Mat> rvectors, tvectors;
	double repError = findArucoMarkers(images, cameraMatrix, distCoeffs, rvectors, tvectors);
	fsCamera.writeCamera(cameraMatrix, distCoeffs);
	cout << "repError = " << repError << endl;
	remapping(images, cameraMatrix, distCoeffs, imageSize, map1, map2);
	findArucoMarkers(images, cameraMatrix, distCoeffs, rvectors, tvectors);
	fstream fin;
	fin.open("/home/jeppe/Hentet/charuco/calib/robot_poses.csv", ios::in);
	vector<string> row;
	string line, word;
	vector<vector<long double>> robotPoses;
	while (robotPoses.size() < 54)
	{
		row.clear();
		vector<long double> poses;
		getline(fin, line);
		stringstream s(line);
		int rowlength =0;
		while(getline(s, word, ',')){
			row.push_back(word);
			rowlength++;
		}
		for	(int i=0; i<rowlength; i++){
			poses.push_back(stold(row.at(i)));
		}
		robotPoses.push_back(poses);
	}
	vector<Mat> rmVec;
	vector<Mat> tmVec;
	for (int i = 0; i < robotPoses.size(); i++)
	{
		DHParams dhp(robotPoses.at(i));
		dhp.calculateDH();
		rmVec.push_back(dhp.getRM());
		Mat tm = dhp.getTM();
		tmVec.push_back(tm);
		pcl::PointXYZ xyz;
		xyz.x = tm.at<double>(0);
		xyz.y = tm.at<double>(1);
		xyz.z = tm.at<double>(2);
		basic_cloud_ptr->push_back(xyz);
	}

	for (int i = 0; i < tvectors.size(); i++)
	{
		pcl::PointXYZ xyz(tvectors.at(i).at<double>(0), tvectors.at(i).at<double>(1),
			tvectors.at(i).at<double>(2));
		basic_cloud_ptr->push_back(xyz);
	}
	Mat cam2GripRM, cam2GripTM;
	calibrateHandEye(rmVec, tmVec, rvectors, tvectors, cam2GripRM, cam2GripTM);
	fsHandEye.writeHandEye(cam2GripRM, cam2GripTM);
	//cout << "camRM: " << endl << cam2GripRM << endl << "camTM: " << endl << cam2GripTM << endl;

	basic_cloud_ptr->width = basic_cloud_ptr->size();
	basic_cloud_ptr->height = 1;

	pcl::visualization::PCLVisualizer::Ptr viewer;

	viewer = viz((pcl::PointCloud<pcl::PointXYZ>::ConstPtr)basic_cloud_ptr);

	cout << "spinnin" << endl;
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		std::this_thread::sleep_for(100ms);
	}
	*/
	return 0;
}

