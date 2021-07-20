#include "dhparams.h"
#include <iterator>

using namespace std;
using namespace cv;

DHParams::DHParams(vector<double> jointVals) {
    _jointVals = jointVals;
}

DHParams::DHParams(vector<vector<double>> robotPoses){
    _robotPoses = robotPoses;
}

void DHParams::calculateDH(){
    std::vector<cv::Mat> rtMatrixVec;
    for (size_t joint = 0; joint < _jointVals.size(); joint++)
    {
        cv::Mat rtMatrix = cv::Mat::zeros(4,4, CV_64F);
        int intSwitch = 0;
        for (int i = 0; i < rtMatrix.rows; i++)
        {
            for (int j = 0; j < rtMatrix.cols; j++)
            {
                switch (intSwitch)
                {
                case (0):
                    rtMatrix.at<double>(i,j) = cos(_jointVals.at(joint));
                    break;
                case (1):
                    rtMatrix.at<double>(i,j) = (-sin(_jointVals.at(joint)))*cos(_alpha.at(joint));
                    break;
                case (2):
                    rtMatrix.at<double>(i,j) = sin(_jointVals.at(joint))*sin(_alpha.at(joint));
                    break;
                case (3):
                    rtMatrix.at<double>(i,j) = _a.at(joint)*cos(_jointVals.at(joint));
                    break;
                case (4):
                    rtMatrix.at<double>(i,j) = sin(_jointVals.at(joint));
                    break;
                case (5):
                    rtMatrix.at<double>(i,j) = cos(_jointVals.at(joint))*cos(_alpha.at(joint));
                    break;
                case (6):
                    rtMatrix.at<double>(i,j) = (cos(_jointVals.at(joint))*sin(_alpha.at(joint)))*(-1);
                    break;
                case (7):
                    rtMatrix.at<double>(i,j) = _a.at(joint)*sin(_jointVals.at(joint));
                    break;
                case (8):
                    rtMatrix.at<double>(i,j) = 0;
                    break;
                case (9):
                    rtMatrix.at<double>(i,j) = sin(_alpha.at(joint));
                    break;
                case (10):
                    rtMatrix.at<double>(i,j) = cos(_alpha.at(joint));
                    break;
                case (11):
                    rtMatrix.at<double>(i,j) = _d.at(joint);
                    break;
                case (12):
                    rtMatrix.at<double>(i,j) = 0;
                    break;
                case (13):
                    rtMatrix.at<double>(i,j) = 0;
                    break;
                case (14):
                    rtMatrix.at<double>(i,j) = 0;
                    break;
                case (15):
                    rtMatrix.at<double>(i,j) = 1;
                    break;
                }
                intSwitch++;
            }
        }
        rtMatrixVec.push_back(rtMatrix);
    }
    _dhParams = multiplyDH(rtMatrixVec);
}

Mat DHParams::calculateAllRM(){
    std::vector<cv::Mat> rtMatrixVec;
    for (size_t poses = 0; poses < _robotPoses.size(); poses++)
    {
        for (size_t joint = 0; joint < _jointVals.size(); joint++)
        {
            cv::Mat rtMatrix = cv::Mat::zeros(4,4, CV_64F);
            int intSwitch = 0;
            for (int i = 0; i < rtMatrix.rows; i++)
            {
                for (int j = 0; j < rtMatrix.cols; j++)
                {
                    switch (intSwitch)
                    {
                    case (0):
                        rtMatrix.at<double>(i,j) = cos(_jointVals.at(joint));
                        break;
                    case (1):
                        rtMatrix.at<double>(i,j) = (-sin(_jointVals.at(joint)))*cos(_alpha.at(joint));
                        break;
                    case (2):
                        rtMatrix.at<double>(i,j) = sin(_jointVals.at(joint))*sin(_alpha.at(joint));
                        break;
                    case (3):
                        rtMatrix.at<double>(i,j) = _a.at(joint)*cos(_jointVals.at(joint));
                        break;
                    case (4):
                        rtMatrix.at<double>(i,j) = sin(_jointVals.at(joint));
                        break;
                    case (5):
                        rtMatrix.at<double>(i,j) = cos(_jointVals.at(joint))*cos(_alpha.at(joint));
                        break;
                    case (6):
                        rtMatrix.at<double>(i,j) = (cos(_jointVals.at(joint))*sin(_alpha.at(joint)))*(-1);
                        break;
                    case (7):
                        rtMatrix.at<double>(i,j) = _a.at(joint)*sin(_jointVals.at(joint));
                        break;
                    case (8):
                        rtMatrix.at<double>(i,j) = 0;
                        break;
                    case (9):
                        rtMatrix.at<double>(i,j) = sin(_alpha.at(joint));
                        break;
                    case (10):
                        rtMatrix.at<double>(i,j) = cos(_alpha.at(joint));
                        break;
                    case (11):
                        rtMatrix.at<double>(i,j) = _d.at(joint);
                        break;
                    case (12):
                        rtMatrix.at<double>(i,j) = 0;
                        break;
                    case (13):
                        rtMatrix.at<double>(i,j) = 0;
                        break;
                    case (14):
                        rtMatrix.at<double>(i,j) = 0;
                        break;
                    case (15):
                        rtMatrix.at<double>(i,j) = 1;
                        break;
                    }
                    intSwitch++;
                }
            }
            rtMatrixVec.push_back(rtMatrix);
        }
        return multiplyDH(rtMatrixVec);
    }
}

Mat DHParams::calculateAllTM(){
    cout << "hej";
    Mat m(1,1,1);
    return m;
}

Mat DHParams::multiplyDH(vector<Mat> rtMatrixVec)
{
    Mat res = Mat::zeros(4,4, CV_64F);
    Mat temp = Mat::zeros(4,4, CV_64F);
    for (vector<Mat>::iterator iter = rtMatrixVec.begin(); iter != rtMatrixVec.end(); iter++)
    {
        if (iter == rtMatrixVec.begin())
        {
            temp = *iter;
        }
        else
        {
            res.at<float>(0)  = (temp.at<float>(0)* (*iter).at<float>(0)) + (temp.at<float>(1)* (*iter).at<float>(4)) + (temp.at<float>(2)* (*iter).at<float>(8)) + (temp.at<float>(3)* (*iter).at<float>(12));
            res.at<float>(1)  = (temp.at<float>(0)* (*iter).at<float>(1)) + (temp.at<float>(1)* (*iter).at<float>(5)) + (temp.at<float>(2)* (*iter).at<float>(9)) + (temp.at<float>(3)* (*iter).at<float>(13));
            res.at<float>(2)  = (temp.at<float>(0)* (*iter).at<float>(2)) + (temp.at<float>(1)* (*iter).at<float>(6)) + (temp.at<float>(2)* (*iter).at<float>(10)) + (temp.at<float>(3)* (*iter).at<float>(14));
            res.at<float>(3)  = (temp.at<float>(0)* (*iter).at<float>(3)) + (temp.at<float>(1)* (*iter).at<float>(7)) + (temp.at<float>(2)* (*iter).at<float>(11)) + (temp.at<float>(3)* (*iter).at<float>(15));
            res.at<float>(4)  = (temp.at<float>(4)* (*iter).at<float>(0)) + (temp.at<float>(5)* (*iter).at<float>(4)) + (temp.at<float>(6)* (*iter).at<float>(8)) + (temp.at<float>(7)* (*iter).at<float>(12));
            res.at<float>(5)  = (temp.at<float>(4)* (*iter).at<float>(1)) + (temp.at<float>(5)* (*iter).at<float>(5)) + (temp.at<float>(6)* (*iter).at<float>(9)) + (temp.at<float>(7)* (*iter).at<float>(13));
            res.at<float>(6)  = (temp.at<float>(4)* (*iter).at<float>(2)) + (temp.at<float>(5)* (*iter).at<float>(6)) + (temp.at<float>(6)* (*iter).at<float>(10)) + (temp.at<float>(7)* (*iter).at<float>(14));
            res.at<float>(7)  = (temp.at<float>(4)* (*iter).at<float>(3)) + (temp.at<float>(5)* (*iter).at<float>(7)) + (temp.at<float>(6)* (*iter).at<float>(11)) + (temp.at<float>(7)* (*iter).at<float>(15));
            res.at<float>(8)  = (temp.at<float>(8)* (*iter).at<float>(0)) + (temp.at<float>(9)* (*iter).at<float>(4)) + (temp.at<float>(10)* (*iter).at<float>(8)) + (temp.at<float>(11)* (*iter).at<float>(12));
            res.at<float>(9)  = (temp.at<float>(8)* (*iter).at<float>(1)) + (temp.at<float>(9)* (*iter).at<float>(5)) + (temp.at<float>(10)* (*iter).at<float>(9)) + (temp.at<float>(11)* (*iter).at<float>(13));
            res.at<float>(10) = (temp.at<float>(8)* (*iter).at<float>(2)) + (temp.at<float>(9)* (*iter).at<float>(6)) + (temp.at<float>(10)* (*iter).at<float>(10)) + (temp.at<float>(11)* (*iter).at<float>(14));
            res.at<float>(11) = (temp.at<float>(8)* (*iter).at<float>(3)) + (temp.at<float>(9)* (*iter).at<float>(7)) + (temp.at<float>(10)* (*iter).at<float>(11)) + (temp.at<float>(11)* (*iter).at<float>(15));
            res.at<float>(12) = (temp.at<float>(12)* (*iter).at<float>(0)) + (temp.at<float>(13)* (*iter).at<float>(4)) + (temp.at<float>(14)* (*iter).at<float>(8)) + (temp.at<float>(15)* (*iter).at<float>(12));
            res.at<float>(13) = (temp.at<float>(12)* (*iter).at<float>(1)) + (temp.at<float>(13)* (*iter).at<float>(5)) + (temp.at<float>(14)* (*iter).at<float>(9)) + (temp.at<float>(15)* (*iter).at<float>(13));
            res.at<float>(14) = (temp.at<float>(12)* (*iter).at<float>(2)) + (temp.at<float>(13)* (*iter).at<float>(6)) + (temp.at<float>(14)* (*iter).at<float>(10)) + (temp.at<float>(15)* (*iter).at<float>(14));
            res.at<float>(15) = (temp.at<float>(12)* (*iter).at<float>(3)) + (temp.at<float>(13)* (*iter).at<float>(7)) + (temp.at<float>(14)* (*iter).at<float>(11)) + (temp.at<float>(15)* (*iter).at<float>(15));

            res.at<double>(0) = (temp.at<double>(0)* (*iter).at<double>(0))+(temp.at<double>(1)* (*iter).at<double>(4))+(temp.at<double>(2)* (*iter).at<double>(8))+(temp.at<double>(3)* (*iter).at<double>(12));
            res.at<double>(1) = (temp.at<double>(0)* (*iter).at<double>(1))+(temp.at<double>(1)* (*iter).at<double>(5))+(temp.at<double>(2)* (*iter).at<double>(9))+(temp.at<double>(3)* (*iter).at<double>(13));
            res.at<double>(2) = (temp.at<double>(0)* (*iter).at<double>(2))+(temp.at<double>(1)* (*iter).at<double>(6))+(temp.at<double>(2)* (*iter).at<double>(10))+(temp.at<double>(3)* (*iter).at<double>(14));
            res.at<double>(3) = (temp.at<double>(0)* (*iter).at<double>(3))+(temp.at<double>(1)* (*iter).at<double>(7))+(temp.at<double>(2)* (*iter).at<double>(11))+(temp.at<double>(3)* (*iter).at<double>(15));
            res.at<double>(4) = (temp.at<double>(4)* (*iter).at<double>(0))+(temp.at<double>(5)* (*iter).at<double>(4))+(temp.at<double>(6)* (*iter).at<double>(8))+(temp.at<double>(7)* (*iter).at<double>(12));
            res.at<double>(5) = (temp.at<double>(4)* (*iter).at<double>(1))+(temp.at<double>(5)* (*iter).at<double>(5))+(temp.at<double>(6)* (*iter).at<double>(9))+(temp.at<double>(7)* (*iter).at<double>(13));
            res.at<double>(6) = (temp.at<double>(4)* (*iter).at<double>(2))+(temp.at<double>(5)* (*iter).at<double>(6))+(temp.at<double>(6)* (*iter).at<double>(10))+(temp.at<double>(7)* (*iter).at<double>(14));
            res.at<double>(7) = (temp.at<double>(4)* (*iter).at<double>(3))+(temp.at<double>(5)* (*iter).at<double>(7))+(temp.at<double>(6)* (*iter).at<double>(11))+(temp.at<double>(7)* (*iter).at<double>(15));
            res.at<double>(8) = (temp.at<double>(8)* (*iter).at<double>(0))+(temp.at<double>(9)* (*iter).at<double>(4))+(temp.at<double>(10)* (*iter).at<double>(8))+(temp.at<double>(11)* (*iter).at<double>(12));
            res.at<double>(9) = (temp.at<double>(8)* (*iter).at<double>(1))+(temp.at<double>(9)* (*iter).at<double>(5))+(temp.at<double>(10)* (*iter).at<double>(9))+(temp.at<double>(11)* (*iter).at<double>(13));
            res.at<double>(10) = (temp.at<double>(8)* (*iter).at<double>(2))+(temp.at<double>(9)* (*iter).at<double>(6))+(temp.at<double>(10)* (*iter).at<double>(10))+(temp.at<double>(11)* (*iter).at<double>(14));
            res.at<double>(11) = (temp.at<double>(8)* (*iter).at<double>(3))+(temp.at<double>(9)* (*iter).at<double>(7))+(temp.at<double>(10)* (*iter).at<double>(11))+(temp.at<double>(11)* (*iter).at<double>(15));
            res.at<double>(12) = (temp.at<double>(12)* (*iter).at<double>(0))+(temp.at<double>(13)* (*iter).at<double>(4))+(temp.at<double>(14)* (*iter).at<double>(8))+(temp.at<double>(15)* (*iter).at<double>(12));
            res.at<double>(13) = (temp.at<double>(12)* (*iter).at<double>(1))+(temp.at<double>(13)* (*iter).at<double>(5))+(temp.at<double>(14)* (*iter).at<double>(9))+(temp.at<double>(15)* (*iter).at<double>(13));
            res.at<double>(14) = (temp.at<double>(12)* (*iter).at<double>(2))+(temp.at<double>(13)* (*iter).at<double>(6))+(temp.at<double>(14)* (*iter).at<double>(10))+(temp.at<double>(15)* (*iter).at<double>(14));
            res.at<double>(15) = (temp.at<double>(12)* (*iter).at<double>(3))+(temp.at<double>(13)* (*iter).at<double>(7))+(temp.at<double>(14)* (*iter).at<double>(11))+(temp.at<double>(15)* (*iter).at<double>(15));
            res.copyTo(temp);
        }
        cout << temp << endl;
    }
    return temp;
}

Mat DHParams::getMatrix()
{
    return _dhParams;
}

Mat DHParams::getRM()
{
    Mat rm = Mat::zeros(3,3, CV_64F);

    rm.at<double>(0) = _dhParams.at<double>(0);
    rm.at<double>(1) = _dhParams.at<double>(1);
    rm.at<double>(2) = _dhParams.at<double>(2);
    rm.at<double>(3) = _dhParams.at<double>(4);
    rm.at<double>(4) = _dhParams.at<double>(5);
    rm.at<double>(5) = _dhParams.at<double>(6);
    rm.at<double>(6) = _dhParams.at<double>(8);
    rm.at<double>(7) = _dhParams.at<double>(9);
    rm.at<double>(8) = _dhParams.at<double>(10);

    return rm;
}

Mat DHParams::getTM()
{
    Mat tm = Mat::zeros(3,1, CV_64F);

    tm.at<double>(0) = _dhParams.at<double>(3);
    tm.at<double>(1) = _dhParams.at<double>(7);
    tm.at<double>(2) = _dhParams.at<double>(11);

    return tm;
}

Mat DHParams::multiplyTM(vector<Mat> rotationMatrixVec)
{
    Mat res = Mat::zeros(3,1, CV_32F);

    return res;
}


DHParams::~DHParams() {
    _robotPoses.clear();
    _jointVals.clear();
}
