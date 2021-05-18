/*
 * FileStorageClass.h
 *
 *  Created on: 6. apr. 2021
 *      Author: jeppe
 */

#ifndef FILESTORAGECLASS_H_
#define FILESTORAGECLASS_H_

#include <opencv2/core.hpp>

#include <string>

using namespace std;
using namespace cv;

class FileStorageClass {
public:
	FileStorageClass(string fileName, string calibrationDate);
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
	virtual ~FileStorageClass();
private:
	string _fileName, _calibrationDate;
};

#endif /* FILESTORAGECLASS_H_ */
