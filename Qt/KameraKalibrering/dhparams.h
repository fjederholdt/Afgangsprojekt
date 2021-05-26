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
    DHParams(std::vector<long double> jointVals);
    DHParams(std::vector<std::vector<long double>> robotPoses);
    void calculateDH();
    cv::Mat calculateAllRM();
    cv::Mat getRM();
    cv::Mat getTM();
    cv::Mat calculateAllTM();
    cv::Mat multiplyDH(std::vector<cv::Mat> rtMatrixVec);
    cv::Mat multiplyTM(std::vector<cv::Mat> translationMatrixVec);
    virtual ~DHParams();
private:
    cv::Mat _dhParams;
    std::vector<std::vector<long double>> _robotPoses;
    std::vector<long double> _jointVals;
    std::vector<long double> _a{0, -0.4250, -0.3922, 0, 0, 0};
    std::vector<long double> _d{0.1625, 0, 0, 0.1333, 0.0997, 0.0996};
    std::vector<long double> _alpha{(M_PI/2), 0, 0, (M_PI/2), (M_PI/2)*(-1), 0};
};


#endif // DHPARAMS_H
