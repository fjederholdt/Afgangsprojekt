#include "kalibreringsFunktioner.h"

using namespace cv;
using namespace std;

vector<double> calibrateCharuco(vector<Mat>& images, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>>& charucoCorners, vector<vector<int>>& charucoIds, vector<Mat>& rotationMat, vector<Mat>& translationMat)
{
    vector<vector<Point2f>> markerCorners;
    vector<int> markerIds;
    Size imageSize = Size(images.at(0).rows, images.at(0).cols);

    params->cornerRefinementMethod = aruco::CORNER_REFINE_APRILTAG;

    cameraMatrix = cameraMatrix12;
    int flags = CALIB_USE_INTRINSIC_GUESS & CALIB_FIX_FOCAL_LENGTH;
    TermCriteria criteria = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 50, DBL_EPSILON);

    for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        Mat inputImage = *iter;
        aruco::detectMarkers(inputImage, board->dictionary, markerCorners, markerIds, params);
        if (markerIds.size() > 0)
        {
            vector<Point2f> arucoCorners;
            vector<int> arucoIds;
            aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImage, board, arucoCorners, arucoIds);
            if(arucoIds.size() > 0){
                charucoIds.push_back(arucoIds);
                charucoCorners.push_back(arucoCorners);
            }
        }
    }

    vector<double> stdDevInt, stdDevExt, repErrors;
    double repError = aruco::calibrateCameraCharuco(charucoCorners, charucoIds, board, imageSize, cameraMatrix, distCoeffs, rotationMat, translationMat, stdDevInt, stdDevExt, repErrors, flags, criteria);
    repErrors.push_back(repError);
    return repErrors;
}

void charucoBoardPose(vector<Mat>& images, Mat& cameraMatrix, Mat& distCoeffs, vector<vector<Point2f>>& charucoCorners, vector<vector<int>>& charucoIds, vector<Mat>& rvectors, vector<Mat>& tvectors)
{
    int i = 0;
    for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        Mat inputImage = *iter;
        aruco::drawDetectedCornersCharuco(inputImage, charucoCorners.at(i), charucoIds.at(i));
        Vec3d rvec, tvec;
        bool valid = aruco::estimatePoseCharucoBoard(charucoCorners.at(i), charucoIds.at(i), board, cameraMatrix, distCoeffs, rvec, tvec);
        if (valid)
        {
            aruco::drawAxis(inputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
            Mat rMat;
            Mat tMat(3,1,CV_64F);
            Rodrigues(rvec, rMat);
            rvectors.push_back(rMat);
            tMat.at <double>(0,0) = tvec[0];
            tMat.at <double>(1,0) = tvec[1];
            tMat.at <double>(2,0) = tvec[2];
            tvectors.push_back(tMat);
        }
        i++;
    }
}

void detectCharuco(vector<Mat>& images, vector<vector<Point2f>>& charucoCorners, vector<vector<int>>& charucoIds)
{
    vector<int> markerIds;
    vector<vector<Point2f>> markerCorners;

    for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        Mat inputImage = *iter;
        aruco::detectMarkers(inputImage, board->dictionary, markerCorners, markerIds, params);
        if (markerIds.size() > 0)
        {
            vector<Point2f> arucoCorners;
            vector<int> arucoIds;
            aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImage, board, arucoCorners, arucoIds);
            charucoIds.push_back(arucoIds);
            charucoCorners.push_back(arucoCorners);
        }
    }
}

void charucoPoints(vector<Mat>& images, Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvectors, vector<Mat>& tvectors)
{
    vector<vector<Point2f>> markerCorners;
    vector<int> markerIds;

    try
    {
        for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
        {
            Mat inputImage = *iter;
            aruco::detectMarkers(inputImage, board->dictionary, markerCorners, markerIds, params);
            if (markerIds.size() > 0)
            {
                vector<Point2f> arucoCorners;
                vector<int> arucoIds;
                aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImage, board, arucoCorners, arucoIds);
                if (arucoIds.size() > 0)
                {
                    Vec3d rvec, tvec;
                    bool valid = aruco::estimatePoseCharucoBoard(arucoCorners, arucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                    if (valid)
                    {
                        Mat rMat;
                        Mat tMat(3,1,CV_64F);
                        Rodrigues(rvec, rMat);
                        rvectors.push_back(rMat);
                        tMat.at <double>(0,0) = tvec[0];
                        tMat.at <double>(1,0) = tvec[1];
                        tMat.at <double>(2,0) = tvec[2];
                        tvectors.push_back(tMat);
                    }
                }
            }
        }
    }  catch (cv::Exception e)
    {
        cout << e.what();
    }

}

void remapping(vector<Mat>& images, const Mat& cameraMatrix, const Mat& distCoeffs, Mat& map1, Mat& map2, vector<Mat>& rview)
{
    Mat view;
    //Size rviewSize = Size(images.at(0).rows, images.at(0).cols);
    Size rviewSize = Size(1440,1080);
    for(size_t i = 0; i < images.size(); i++)
    {
        rview.push_back(view);
    }
    // setup enlargement and offset for new image
    /*double y_shift = 60;
    double x_shift = 70;

    rviewSize.height += 2*y_shift;
    rviewSize.width += 2*x_shift;

    // create a new camera matrix with the principal point
    // offest according to the offset above
    Mat newCameraMatrix = cameraMatrix.clone();

    newCameraMatrix.at<double>(0, 2) += x_shift; //adjust c_x by x_shift
    newCameraMatrix.at<double>(1, 2) += y_shift; //adjust c_y by y_shift
*/
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), cameraMatrix, rviewSize, CV_8UC1, map1, map2);

    int i = 0;

    for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        view = *iter;
        remap(view, rview.at(i), map1, map2, INTER_LINEAR);
        i++;
    }


    /*
    for (int i=0; i< rview.size(); i++){
    namedWindow("Undistorted Map", 1);
    imshow("Undistorted", rview.at(i));
    int k = waitKey(0); // Wait for a keystroke in the window
    }
*/

}

vector<Mat> getImages(vector<string> paths)
{
    vector<Mat> images;
    for (size_t i = 0; i < paths.size(); i++)
    {
        Mat img = imread(paths.at(i), IMREAD_GRAYSCALE);

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

















