/*
 * FileStorageClass.cpp
 *
 *  Created on: 6. apr. 2021
 *      Author: jeppe
 */

#include "FileStorageClass.h"

FileStorageClass::FileStorageClass(string fileName, string calibrationDate) {
	_fileName = fileName;
	_calibrationDate = calibrationDate;
}

void FileStorageClass::writeCamera(Mat cameraMatrix, Mat distCoeffs){
	FileStorage fs(_fileName, FileStorage::WRITE);
	fs << "calibrationDate" << _calibrationDate;
	fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;
	fs.release();
}

void FileStorageClass::writeRobot(Mat rotationMatrix, Mat translationMatrix){
	FileStorage fs(_fileName, FileStorage::WRITE);
	fs << "calibrationDate" << _calibrationDate;
	fs << "robotRotationMatrix" << rotationMatrix << "robotTranslationMatrix" << translationMatrix;
	fs.release();
}

void FileStorageClass::writeHandEye(Mat rotationMatrix, Mat translationMatrix){
	FileStorage fs(_fileName, FileStorage::WRITE);
	fs << "calibrationDate" << _calibrationDate;
	fs << "handEyeRotationMatrix" << rotationMatrix << "handEyeTranslationMatrix" << translationMatrix;
	fs.release();
}

void FileStorageClass::writeDate(string calibrationDate){
	FileStorage fs(_fileName, FileStorage::WRITE);
	fs << "calibrationDate" << calibrationDate;
	fs.release();
}

void FileStorageClass::write(Mat data, string type){
	FileStorage fs(_fileName, FileStorage::WRITE);
	Size dataSize = data.size();
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

void FileStorageClass::readCamera(Mat cameraMatrix, Mat distCoeffs){
	FileStorage fs(_fileName, FileStorage::READ);
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;
	fs.release();
}

void FileStorageClass::readRobot(Mat rotationMatrix, Mat translationMatrix){
	FileStorage fs(_fileName, FileStorage::READ);
	fs["robotRotationMatrix"] >> rotationMatrix;
	fs["robotTranslationMatrix"] >> translationMatrix;
	fs.release();
}

void FileStorageClass::readHandEye(Mat rotationMatrix, Mat translationMatrix){
	FileStorage fs(_fileName, FileStorage::READ);
	fs["handEyeRotationMatrix"] >> rotationMatrix;
	fs["handEyeTranslationMatrix"] >> translationMatrix;
	fs.release();
}

void FileStorageClass::readDate(string date){
	FileStorage fs(_fileName, FileStorage::READ);
	fs["calibrationDate"] >> date;
	fs.release();
}

void FileStorageClass::read(Mat cameraMatrix, Mat distCoeffs){
	FileStorage fs(_fileName, FileStorage::READ);
	fs["cameraMatrix"] >> cameraMatrix;
	fs["distCoeffs"] >> distCoeffs;
	fs.release();
}

FileStorageClass::~FileStorageClass() {
	// TODO Auto-generated destructor stub
}

