#ifndef FSCLASS_H
#define FSCLASS_H

#include <opencv2/core.hpp>

#include <string>

using namespace std;
using namespace cv;

class FSClass
{
public:
    FSClass(string fileName, string calibrationDate);
    void writeCamera(Mat cameraMatrix, Mat distCoeffs);
    void writeRobot(Mat rotationMatrix, Mat translationMatrix);
    void writeHandEye(Mat rotationMatrix, Mat translationMatrix);
    void writeDate(string date);
    void write(Mat data, string type);
    void readCamera(Mat cameraMatrix, Mat distCoeffs);
    void readRobot(Mat rotationMatrix, Mat translationMatrix);
    void readHandEye(Mat rotationMatrix, Mat translationMatrix);
    void readDate(string date);
    void read(Mat cameraMatrix, Mat distCoeffs);
    virtual ~FSClass();
private:
    string _fileName, _calibrationDate;
};

#endif // FSCLASS_H
