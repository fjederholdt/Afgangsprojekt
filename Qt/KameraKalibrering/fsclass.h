#ifndef FSCLASS_H
#define FSCLASS_H

#include <opencv2/core.hpp>

#include <string>

class FSClass
{
public:
    FSClass(std::string fileName, std::string calibrationDate);
    void writeCamera(cv::Mat cameraMatrix, cv::Mat distCoeffs);
    void writeRobot(cv::Mat rotationMatrix, cv::Mat translationMatrix);
    void writeHandEye(cv::Mat rotationMatrix, cv::Mat translationMatrix);
    void writeDate(std::string date);
    void write(cv::Mat data, std::string type);
    void readCamera(cv::Mat cameraMatrix, cv::Mat distCoeffs);
    void readRobot(cv::Mat rotationMatrix, cv::Mat translationMatrix);
    void readHandEye(cv::Mat rotationMatrix, cv::Mat translationMatrix);
    void readDate(std::string date);
    void read(cv::Mat cameraMatrix, cv::Mat distCoeffs);
    virtual ~FSClass();
private:
    std::string _fileName, _calibrationDate;
};

#endif // FSCLASS_H
