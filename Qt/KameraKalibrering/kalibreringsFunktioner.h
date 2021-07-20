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

    std::vector<double> calibrateCharuco(std::vector<cv::Mat>& images, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<std::vector<cv::Point2f>>& charucoCorners, std::vector<std::vector<int>>& charucoIds, std::vector<cv::Mat>& rotationMat, std::vector<cv::Mat>& translationMat);
    void charucoBoardPose(std::vector<cv::Mat>& images, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<std::vector<cv::Point2f>>& charucoCorners, std::vector<std::vector<int>>& charucoIds, std::vector<cv::Mat>& rvectors, std::vector<cv::Mat>& tvectors);
    void detectCharuco(std::vector<cv::Mat>& images, std::vector<std::vector<cv::Point2f>>& charucoCorners, std::vector<std::vector<int>>& charucoIds);
    void charucoPoints(std::vector<cv::Mat>& images, cv::Mat& cameraMatrix, cv::Mat& distCoeffs, std::vector<cv::Mat>& rvectors, std::vector<cv::Mat>& tvectors);
    void remapping(std::vector<cv::Mat>& images, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, cv::Mat& map1, cv::Mat& map2, std::vector<cv::Mat>& rview);
    std::vector<cv::Mat> getImages(std::vector<std::string> paths);
    void caliSharpnes(std::vector<cv::Mat> rview);

    static const float chessSquareDim = 0.02f;
    static const float arucoSquareDim = 0.015f;
    static const cv::Size chessboardDim = cv::Size(9, 14);

    static  cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    static const cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100);
    static const cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(chessboardDim.height, chessboardDim.width, chessSquareDim, arucoSquareDim, dictionary);

    static double matrix12[9] = {3478.26089, 0, 720, 0, 3478.26089, 540, 0, 0, 1};
    static const cv::Mat cameraMatrix12(3, 3, 6, matrix12);

#endif // KALIBRERINGSFUNKTIONER_H
