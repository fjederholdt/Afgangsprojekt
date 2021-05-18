/*
 * DHParams.cpp
 *
 *  Created on: 14. apr. 2021
 *      Author: jeppe
 */

#include "DHParams.h"
#include <iterator>

using namespace std;
using namespace cv;

DHParams::DHParams(vector<long double> jointVals) {
	_jointVals = jointVals;
	/*for (vector<float>::iterator iter=_jointVals.begin(); iter!=_jointVals.end(); iter++)
	{
		cout << "joint values: " << endl << *iter << endl;
	}*/
}

DHParams::DHParams(vector<vector<long double>> robotPoses){
	_robotPoses = robotPoses;
}

void DHParams::calculateDH(){
	std::vector<cv::Mat> rtMatrixVec;
	for (int joint = 0; joint < (int)_jointVals.size(); joint++)
	{
		cv::Mat rtMatrix = cv::Mat::zeros(4,4, CV_32F);
		int intSwitch = 0;
		for (int i = 0; i < rtMatrix.rows; i++)
		{
			for (int j = 0; j < rtMatrix.cols; j++)
			{
				switch (intSwitch)
				{
					case (0):
						rtMatrix.at<float>(i,j) = cos(_jointVals.at(joint));
						break;
					case (1):
						rtMatrix.at<float>(i,j) = (-sin(_jointVals.at(joint)))*cos(_alpha.at(joint));
						break;
					case (2):
						rtMatrix.at<float>(i,j) = sin(_jointVals.at(joint))*sin(_alpha.at(joint));
						break;
					case (3):
						rtMatrix.at<float>(i,j) = _a.at(joint)*cos(_jointVals.at(joint));
						break;
					case (4):
						rtMatrix.at<float>(i,j) = sin(_jointVals.at(joint));
						break;
					case (5):
						rtMatrix.at<float>(i,j) = cos(_jointVals.at(joint))*cos(_alpha.at(joint));
						break;
					case (6):
						rtMatrix.at<float>(i,j) = (cos(_jointVals.at(joint))*sin(_alpha.at(joint)))*(-1);
						break;
					case (7):
						rtMatrix.at<float>(i,j) = _a.at(joint)*sin(_jointVals.at(joint));
						break;
					case (8):
						rtMatrix.at<float>(i,j) = 0;
						break;
					case (9):
						rtMatrix.at<float>(i,j) = sin(_alpha.at(joint));
						break;
					case (10):
						rtMatrix.at<float>(i,j) = cos(_alpha.at(joint));
						break;
					case (11):
						rtMatrix.at<float>(i,j) = _d.at(joint);
						break;
					case (12):
						rtMatrix.at<float>(i,j) = 0;
						break;
					case (13):
						rtMatrix.at<float>(i,j) = 0;
						break;
					case (14):
						rtMatrix.at<float>(i,j) = 0;
						break;
					case (15):
						rtMatrix.at<float>(i,j) = 1;
						break;
				}
				intSwitch++;
			}
		}
		//cout << "rm" << endl << _rotationMatrix << endl;
		rtMatrixVec.push_back(rtMatrix);
		/*for (vector<Mat>::iterator iter=rotationMatrixVec.begin(); iter!=rotationMatrixVec.end(); iter++){

			cout << *iter << endl;
		}
		cout << "next" << endl;*/
	}
	_dhParams = multiplyDH(rtMatrixVec);
}

Mat DHParams::calculateAllRM(){
	std::vector<cv::Mat> rtMatrixVec;
	for (int poses = 0; poses < (int)_robotPoses.size(); poses++)
	{
		for (int joint = 0; joint < (int)_jointVals.size(); joint++)
		{
			cv::Mat rtMatrix = cv::Mat::zeros(4,4, CV_32F);
			int intSwitch = 0;
			for (int i = 0; i < rtMatrix.rows; i++)
			{
				for (int j = 0; j < rtMatrix.cols; j++)
				{
					switch (intSwitch)
					{
						case (0):
							rtMatrix.at<float>(i,j) = cos(_jointVals.at(joint));
							break;
						case (1):
							rtMatrix.at<float>(i,j) = (-sin(_jointVals.at(joint)))*cos(_alpha.at(joint));
							break;
						case (2):
							rtMatrix.at<float>(i,j) = sin(_jointVals.at(joint))*sin(_alpha.at(joint));
							break;
						case (3):
							rtMatrix.at<float>(i,j) = _a.at(joint)*cos(_jointVals.at(joint));
							break;
						case (4):
							rtMatrix.at<float>(i,j) = sin(_jointVals.at(joint));
							break;
						case (5):
							rtMatrix.at<float>(i,j) = cos(_jointVals.at(joint))*cos(_alpha.at(joint));
							break;
						case (6):
							rtMatrix.at<float>(i,j) = (cos(_jointVals.at(joint))*sin(_alpha.at(joint)))*(-1);
							break;
						case (7):
							rtMatrix.at<float>(i,j) = _a.at(joint)*sin(_jointVals.at(joint));
							break;
						case (8):
							rtMatrix.at<float>(i,j) = 0;
							break;
						case (9):
							rtMatrix.at<float>(i,j) = sin(_alpha.at(joint));
							break;
						case (10):
							rtMatrix.at<float>(i,j) = cos(_alpha.at(joint));
							break;
						case (11):
							rtMatrix.at<float>(i,j) = _d.at(joint);
							break;
						case (12):
							rtMatrix.at<float>(i,j) = 0;
							break;
						case (13):
							rtMatrix.at<float>(i,j) = 0;
							break;
						case (14):
							rtMatrix.at<float>(i,j) = 0;
							break;
						case (15):
							rtMatrix.at<float>(i,j) = 1;
							break;
					}
					intSwitch++;
				}
			}
			//cout << "rm" << endl << _rotationMatrix << endl;
			rtMatrixVec.push_back(rtMatrix);
			/*for (vector<Mat>::iterator iter=rotationMatrixVec.begin(); iter!=rotationMatrixVec.end(); iter++){

				cout << *iter << endl;
			}
			cout << "next" << endl;*/
		}
		return multiplyDH(rtMatrixVec);
	}
}

Mat DHParams::calculateAllTM(){
	cout << "hej";
}

Mat DHParams::multiplyDH(vector<Mat> rtMatrixVec)
{
	Mat res = Mat::zeros(4,4, CV_32F);
	Mat temp = Mat::zeros(4,4, CV_32F);
	for (vector<Mat>::iterator iter = rtMatrixVec.begin(); iter != rtMatrixVec.end(); iter++)
	{
		if (iter == rtMatrixVec.begin())
		{
			temp = *iter;
		}
		else
		{
			res.at<float>(0) = (temp.at<float>(0)* (*iter).at<float>(0))+(temp.at<float>(1)* (*iter).at<float>(4))+(temp.at<float>(2)* (*iter).at<float>(8))+(temp.at<float>(3)* (*iter).at<float>(12));
			res.at<float>(1) = (temp.at<float>(0)* (*iter).at<float>(1))+(temp.at<float>(1)* (*iter).at<float>(5))+(temp.at<float>(2)* (*iter).at<float>(9))+(temp.at<float>(3)* (*iter).at<float>(13));
			res.at<float>(2) = (temp.at<float>(0)* (*iter).at<float>(2))+(temp.at<float>(1)* (*iter).at<float>(6))+(temp.at<float>(2)* (*iter).at<float>(10))+(temp.at<float>(3)* (*iter).at<float>(14));
			res.at<float>(3) = (temp.at<float>(0)* (*iter).at<float>(3))+(temp.at<float>(1)* (*iter).at<float>(7))+(temp.at<float>(2)* (*iter).at<float>(11))+(temp.at<float>(3)* (*iter).at<float>(15));
			res.at<float>(4) = (temp.at<float>(4)* (*iter).at<float>(0))+(temp.at<float>(5)* (*iter).at<float>(4))+(temp.at<float>(6)* (*iter).at<float>(8))+(temp.at<float>(7)* (*iter).at<float>(12));
			res.at<float>(5) = (temp.at<float>(4)* (*iter).at<float>(1))+(temp.at<float>(5)* (*iter).at<float>(5))+(temp.at<float>(6)* (*iter).at<float>(9))+(temp.at<float>(7)* (*iter).at<float>(13));
			res.at<float>(6) = (temp.at<float>(4)* (*iter).at<float>(2))+(temp.at<float>(5)* (*iter).at<float>(6))+(temp.at<float>(6)* (*iter).at<float>(10))+(temp.at<float>(7)* (*iter).at<float>(14));
			res.at<float>(7) = (temp.at<float>(4)* (*iter).at<float>(3))+(temp.at<float>(5)* (*iter).at<float>(7))+(temp.at<float>(6)* (*iter).at<float>(11))+(temp.at<float>(7)* (*iter).at<float>(15));
			res.at<float>(8) = (temp.at<float>(8)* (*iter).at<float>(0))+(temp.at<float>(9)* (*iter).at<float>(4))+(temp.at<float>(10)* (*iter).at<float>(8))+(temp.at<float>(11)* (*iter).at<float>(12));
			res.at<float>(9) = (temp.at<float>(8)* (*iter).at<float>(1))+(temp.at<float>(9)* (*iter).at<float>(5))+(temp.at<float>(10)* (*iter).at<float>(9))+(temp.at<float>(11)* (*iter).at<float>(13));
			res.at<float>(10) = (temp.at<float>(8)* (*iter).at<float>(2))+(temp.at<float>(9)* (*iter).at<float>(6))+(temp.at<float>(10)* (*iter).at<float>(10))+(temp.at<float>(11)* (*iter).at<float>(14));
			res.at<float>(11) = (temp.at<float>(8)* (*iter).at<float>(3))+(temp.at<float>(9)* (*iter).at<float>(7))+(temp.at<float>(10)* (*iter).at<float>(11))+(temp.at<float>(11)* (*iter).at<float>(15));
			res.at<float>(12) = (temp.at<float>(12)* (*iter).at<float>(0))+(temp.at<float>(13)* (*iter).at<float>(4))+(temp.at<float>(14)* (*iter).at<float>(8))+(temp.at<float>(15)* (*iter).at<float>(12));
			res.at<float>(13) = (temp.at<float>(12)* (*iter).at<float>(1))+(temp.at<float>(13)* (*iter).at<float>(5))+(temp.at<float>(14)* (*iter).at<float>(9))+(temp.at<float>(15)* (*iter).at<float>(13));
			res.at<float>(14) = (temp.at<float>(12)* (*iter).at<float>(2))+(temp.at<float>(13)* (*iter).at<float>(6))+(temp.at<float>(14)* (*iter).at<float>(10))+(temp.at<float>(15)* (*iter).at<float>(14));
			res.at<float>(15) = (temp.at<float>(12)* (*iter).at<float>(3))+(temp.at<float>(13)* (*iter).at<float>(7))+(temp.at<float>(14)* (*iter).at<float>(11))+(temp.at<float>(15)* (*iter).at<float>(15));
			res.copyTo(temp);
		}
    }
    return temp;
}

Mat DHParams::getRM()
{
	Mat rm = Mat::zeros(3,3, CV_32F);

	rm.at<float>(0) = _dhParams.at<float>(0);
	rm.at<float>(1) = _dhParams.at<float>(1);
	rm.at<float>(2) = _dhParams.at<float>(2);
	rm.at<float>(3) = _dhParams.at<float>(4);
	rm.at<float>(4) = _dhParams.at<float>(5);
	rm.at<float>(5) = _dhParams.at<float>(6);
	rm.at<float>(6) = _dhParams.at<float>(8);
	rm.at<float>(7) = _dhParams.at<float>(9);
	rm.at<float>(8) = _dhParams.at<float>(10);

	return rm;
}

Mat DHParams::getTM()
{
	Mat tm = Mat::zeros(3,1, CV_32F);

	tm.at<float>(0) = _dhParams.at<float>(3);
	tm.at<float>(1) = _dhParams.at<float>(7);
	tm.at<float>(2) = _dhParams.at<float>(11);

	return tm;
}

Mat DHParams::multiplyTM(vector<Mat> rotationMatrixVec)
{
	Mat res = Mat::zeros(3,1, CV_32F);
    /*int i, j, k;
    for (i = 0; i < 3; i++) {
        for (j = 0; j < 1; j++) {
            res[i][j] = 0;
            for (k = 0; k < N; k++)
                res[i][j] += mat1[i][k] * mat2[k][j];
        }
    }*/
    return res;
}


DHParams::~DHParams() {
	_robotPoses.clear();
	_jointVals.clear();
}

