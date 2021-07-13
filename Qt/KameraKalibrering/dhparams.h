#ifndef DHPARAMS_H
#define DHPARAMS_H

#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <vector>
#include <math.h>
#include <cstdio>
#include <iostream>

class DHParams
{
public:
    DHParams(std::vector<double> jointVals);
    DHParams(std::vector<std::vector<double>> robotPoses);
    void calculateDH();
    cv::Mat calculateAllRM();
    cv::Mat getMatrix();
    cv::Mat getRM();
    cv::Mat getTM();
    cv::Mat calculateAllTM();
    cv::Mat multiplyDH(std::vector<cv::Mat> rtMatrixVec);
    cv::Mat multiplyTM(std::vector<cv::Mat> translationMatrixVec);
    virtual ~DHParams();
private:
    cv::Mat _dhParams;
    std::vector<std::vector<double>> _robotPoses;
    std::vector<double> _jointVals;
    std::vector<double> _a{0, -0.425, -0.39225, 0, 0, 0};
    std::vector<double> _d{0.089159, 0, 0, 0.10915, 0.09456, 0.0823};
    std::vector<double> _alpha{(M_PI/2), 0, 0, (M_PI/2), (M_PI/2)*(-1), 0};
};


#endif // DHPARAMS_H
