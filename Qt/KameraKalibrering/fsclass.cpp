#include "fsclass.h"

FSClass::FSClass(string fileName, string calibrationDate) {
    _fileName = fileName;
    _calibrationDate = calibrationDate;
}

void FSClass::writeCamera(Mat cameraMatrix, Mat distCoeffs){
    FileStorage fs(_fileName, FileStorage::WRITE);
    fs << "calibrationDate" << _calibrationDate;
    fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
    fs.release();
}

void FSClass::writeRobot(Mat rotationMatrix, Mat translationMatrix){
    FileStorage fs(_fileName, FileStorage::WRITE);
    fs << "calibrationDate" << _calibrationDate;
    fs << "robotRotationMatrix" << rotationMatrix << "robotTranslationMatrix" << translationMatrix;
    fs.release();
}

void FSClass::writeHandEye(Mat rotationMatrix, Mat translationMatrix){
    FileStorage fs(_fileName, FileStorage::WRITE);
    fs << "calibrationDate" << _calibrationDate;
    fs << "handEyeRotationMatrix" << rotationMatrix << "handEyeTranslationMatrix" << translationMatrix;
    fs.release();
}

void FSClass::writeDate(string calibrationDate){
    FileStorage fs(_fileName, FileStorage::WRITE);
    fs << "calibrationDate" << calibrationDate;
    fs.release();
}

void FSClass::write(Mat data, string type){
    FileStorage fs(_fileName, FileStorage::WRITE);
    if (type == "cameraMatrix")
        fs << "cameraMatrix" << data;
    else if (type == "distCoeffs")
        fs << "distCoeffs" << data;
    else if (type == "robotRotationMatrix")
        fs << "robotRotationMatrix" << data;
    else if (type == "robotTranslationMatrix")
        fs << "robotTranslationMatrix" << data;
    else if (type == "handEyeRotationMatrix")
        fs << "handEyeRotationMatrix" << data;
    else if (type == "handEyeTranslationMatrix")
        fs << "handEyeTranslationMatrix" << data;
}

void FSClass::readCamera(Mat cameraMatrix, Mat distCoeffs){
    FileStorage fs(_fileName, FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();
}

void FSClass::readRobot(Mat rotationMatrix, Mat translationMatrix){
    FileStorage fs(_fileName, FileStorage::READ);
    fs["robotRotationMatrix"] >> rotationMatrix;
    fs["robotTranslationMatrix"] >> translationMatrix;
    fs.release();
}

void FSClass::readHandEye(Mat rotationMatrix, Mat translationMatrix){
    FileStorage fs(_fileName, FileStorage::READ);
    fs["handEyeRotationMatrix"] >> rotationMatrix;
    fs["handEyeTranslationMatrix"] >> translationMatrix;
    fs.release();
}

void FSClass::readDate(string date){
    FileStorage fs(_fileName, FileStorage::READ);
    fs["calibrationDate"] >> date;
    fs.release();
}

void FSClass::read(Mat cameraMatrix, Mat distCoeffs){
    FileStorage fs(_fileName, FileStorage::READ);
    fs["cameraMatrix"] >> cameraMatrix;
    fs["distCoeffs"] >> distCoeffs;
    fs.release();
}

FSClass::~FSClass() {
    // TODO Auto-generated destructor stub
}
