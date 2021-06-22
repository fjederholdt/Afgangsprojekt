#include "kalibreringsFunktioner.h"

using namespace cv;
using namespace std;

const float chessSquareDim = 0.02f;
const float arucoSquareDim = 0.015f;
const Size chessboardDim = Size(9, 14);

double calibrateCharuco(vector<Mat>& images, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>>& charucoCorners, vector<vector<int>>& charucoIds)
{
    Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100);
    Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(chessboardDim.height, chessboardDim.width, chessSquareDim, arucoSquareDim, dictionary);

    vector<int> markerIds;
    vector<vector<Point2f>> markerCorners;
    vector<Mat> rvecs, tvecs;
    Size imageSize = Size(images.at(0).rows, images.at(0).cols);

    for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        Mat inputImage = *iter;
        aruco::detectMarkers(inputImage, board->dictionary, markerCorners, markerIds);
        if (markerIds.size() > 0)
        {
            vector<Point2f> arucoCorners;
            vector<int> arucoIds;
            aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImage, board, arucoCorners, arucoIds);
            charucoIds.push_back(arucoIds);
            charucoCorners.push_back(arucoCorners);
        }
    }

    return aruco::calibrateCameraCharuco(charucoCorners, charucoIds, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
}

void CharucoBoardPose(vector<Mat>& images, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>>& charucoCorners, vector<vector<int>>& charucoIds, vector<Mat>& rvectors, vector<Mat>& tvectors)
{
    Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100);
    Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(chessboardDim.height, chessboardDim.width, chessSquareDim, arucoSquareDim, dictionary);
    int i = 0;
    for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        Mat inputImage = *iter;
        aruco::drawDetectedCornersCharuco(inputImage, charucoCorners.at(i), charucoIds.at(i));
        Vec3d rvec, tvec;
        Mat rMat, tMat;
        bool valid = aruco::estimatePoseCharucoBoard(charucoCorners.at(i), charucoIds.at(i), board, cameraMatrix, distCoeffs, rvec, tvec);
        if (valid)
        {
            aruco::drawAxis(inputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
            Rodrigues(rvec, rMat);
            Rodrigues(tvec, tMat);
            rvectors.push_back(rMat);
            tvectors.push_back(tMat);
        }
        i++;
    }
}

void remapping(vector<Mat>& images, const Mat& cameraMatrix, const Mat& distCoeffs, Mat& map1, Mat& map2, vector<Mat>& rview){
    Mat view;

    // setup enlargement and offset for new image
    double y_shift = 60;
    double x_shift = 70;
    Size rviewSize = Size(images.at(0).rows, images.at(0).cols);
    rviewSize.height += 2*y_shift;
    rviewSize.width += 2*x_shift;

    // create a new camera matrix with the principal point
    // offest according to the offset above
    Mat newCameraMatrix = cameraMatrix.clone();
    for(size_t i = 0; i < images.size(); i++)
    {
        rview.push_back(view);
    }
    newCameraMatrix.at<double>(0, 2) += x_shift; //adjust c_x by x_shift
    newCameraMatrix.at<double>(1, 2) += y_shift; //adjust c_y by y_shift


    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), newCameraMatrix, rviewSize, CV_8UC1, map1, map2);
    //initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_8UC1, map1, map2);
    int i = 0;
    for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        view = *iter;
        remap(view, rview.at(i), map1, map2, INTER_LINEAR);
        i++;
    }
    //Print undistorted billeder og find corners til sharpness
    //namedWindow("Undistorted Map", 1);
    //imshow("Undistorted", rview);
    //int k = waitKey(0); // Wait for a keystroke in the window
}

vector<Mat> getImages(vector<string> paths){
    vector<Mat> images;
    for (size_t i = 0; i < paths.size(); i++)
    {
        Mat img = imread(paths.at(i),IMREAD_GRAYSCALE);
        string sub;
        size_t found = paths.at(i).find_last_of("/");
        sub = paths.at(i).substr(found+1, paths.at(i).size());
        if (!img.empty())
        {
            images.push_back(img);
            cout << "found image: " << sub << endl;
        }
        else
            cout << "Didn't find image: " << sub << endl;
    }
    return images;
}

















