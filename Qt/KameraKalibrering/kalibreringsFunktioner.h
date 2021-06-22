#ifndef KALIBRERINGSFUNKTIONER_H
#define KALIBRERINGSFUNKTIONER_H

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>
#include <string>

    std::vector<double> calibrateCharuco(std::vector<cv::Mat>& images, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<std::vector<cv::Point2f>>& charucoCorners, std::vector<std::vector<int>>& charucoIds);
    void CharucoBoardPose(std::vector<cv::Mat>& images, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<std::vector<cv::Point2f>>& charucoCorners, std::vector<std::vector<int>>& charucoIds, std::vector<cv::Mat>& rvectors, std::vector<cv::Mat>& tvectors);
    void remapping(std::vector<cv::Mat>& images, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,const cv::Size imageSize, cv::Mat& map1, cv::Mat& map2);
    std::vector<cv::Mat> getImages(std::vector<std::string> paths);

#endif // KALIBRERINGSFUNKTIONER_H
