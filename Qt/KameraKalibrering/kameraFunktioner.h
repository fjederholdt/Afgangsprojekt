#ifndef KAMERAFUNKTIONER_H
#define KAMERAFUNKTIONER_H

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>

#include <pylon/PylonBase.h>
#include <pylon/PylonIncludes.h>
#include <pylon/TlFactory.h>
#include <pylon/ImageFormatConverter.h>
#include <pylon/PylonImage.h>

void tagBillede(cv::Mat& grayImage, bool show);

static int cameraExposure = 70000;

#endif // KAMERAFUNKTIONER_H
