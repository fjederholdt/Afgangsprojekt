#include "nykalibrering.h"
#include "ui_nykalibrering.h"
#include "kalibrering.h"

using namespace std::filesystem;
using namespace std;
using namespace cv;

const float chessSquareDim = 0.02f;
const float arucoSquareDim = 0.015f;
const Size chessboardDim = Size(9, 14);

NyKalibrering::NyKalibrering(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::NyKalibrering)
{
    ui->setupUi(this);
    time_t rawtime;
    time(&rawtime);
    localTime = asctime(localtime(&rawtime));
    path = "/home/jeppe/Qt-workspace/KameraKalibrering/Kalibreringer/"+localTime;
    path.pop_back();
    mode_t mode = 0755;
    char *copypath = strdup(path.c_str());
    if(mkdir(copypath, mode) != 0 && errno != EEXIST)
    {
        std::cerr << "Error : " << strerror(errno) << std::endl;
    }
    cam.open(0,CAP_ANY);
    /*std::vector<std::string> pathVector;
    std::string sub, suffix;
    for(const auto & entry : directory_iterator(path))
    {
        sub = entry.path();
        std::size_t found = sub.find_last_of("/");
        sub = sub.substr(found+1,sub.size());
        std::size_t suffixFound = sub.find_last_of(".");
        suffix = sub.substr(suffixFound,sub.size());
        if(suffix == ".png" || suffix == ".jpg" || suffix == ".jpeg" )
            pathVector.push_back(sub);
    }
    for (size_t i = 0; i < pathVector.size(); i++)
    {
        QListWidgetItem *item = new QListWidgetItem;
        item->setText(QString::fromStdString(pathVector.at(i)));
        item->setCheckState(Qt::Unchecked);
        ui->listWidget->addItem(item);
    }*/
}

NyKalibrering::~NyKalibrering()
{
    delete ui;
}

void NyKalibrering::addList(std::string strItem)
{
    QListWidgetItem *item = new QListWidgetItem;
    item->setText(QString::fromStdString(strItem));
    ui->listWidget->addItem(item);
}

void NyKalibrering::removeList()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    foreach(QListWidgetItem* item, items)
        delete ui->listWidget->takeItem(ui->listWidget->row(item));
}

void NyKalibrering::on_tag_billede_clicked()
{
    if(!cam.isOpened())
        std::cout << "no camera" << std::endl;
    Mat cam_img, save_img;
    namedWindow("Camera");
    for (; ; ) {
        cam >> cam_img;
        if(cam_img.empty())
            break;

        imshow("Camera", cam_img);

        char c = (char)waitKey(1);
        if (c == 27) break;
    }
    destroyAllWindows();
    cvtColor(cam_img, save_img, COLOR_RGB2GRAY);
    imwrite(path+"/"+to_string(image_nr)+".png", save_img);
    addList(to_string(image_nr)+".png");
    image_nr++;
}


void NyKalibrering::on_slet_billede_clicked()
{
    vector<std::string> pathVector;
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    for(int i = 0; i < items.size(); i++)
       pathVector.push_back(path+"/"+items.at(i)->text().toStdString());
    for(size_t i = 0; i < pathVector.size(); i++)
        remove(pathVector.at(i));
    removeList();
}

void NyKalibrering::on_valg_alle_billeder_clicked()
{
    ui->listWidget->selectAll();
}


void NyKalibrering::on_brug_vs_clicked()
{

}

double findArucoMarkers(vector<Mat>& images, Mat& cameraMatrix, Mat& distCoeffs, vector<Mat>& rvectors, vector<Mat>& tvectors)
{
    try
    {
    Ptr<aruco::DetectorParameters> params = aruco::DetectorParameters::create();
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME::DICT_4X4_100);
    Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(chessboardDim.height, chessboardDim.width, chessSquareDim, arucoSquareDim, dictionary);
    vector<int> markerIds;
    vector<vector<int>> markerIdsVec;
    vector<vector<Point2f>> markerCorners;
    vector<vector<Point2f>> markerCornersVec;
    vector<Mat> rvecs, tvecs;
    Size imageSize = Size(images.at(0).rows, images.at(0).cols);
    Mat boardImage;
    board->draw(Size(600, 400), boardImage, 1, 1);
    //imshow("Board Image", boardImage);
    //waitKey(0);

    for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        Mat inputImage = *iter;
        aruco::detectMarkers(inputImage, board->dictionary, markerCorners, markerIds);
        if (markerIds.size() > 0)
        {
            vector<Point2f> charucoCorners;
            vector<int> charucoIds;
            aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImage, board, charucoCorners, charucoIds);
            markerIdsVec.push_back(charucoIds);
            markerCornersVec.push_back(charucoCorners);
            if(charucoIds.size() > 0 && !cameraMatrix.empty())
            {
                aruco::drawDetectedCornersCharuco(inputImage, charucoCorners, charucoIds);
                Vec3d rvec, tvec;
                Mat rMat, tMat;
                bool valid = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec, tvec);
                if (valid)
                {
                    aruco::drawAxis(inputImage, cameraMatrix, distCoeffs, rvec, tvec, 0.1);
                    //imshow("charuco", inputImage);
                    Rodrigues(rvec, rMat);
                    Rodrigues(tvec, tMat);
                    rvectors.push_back(rMat);
                    tvectors.push_back(tMat);
                    //cout << "rvec: " << endl << rvec << endl << "tvec: " << endl << tvec << endl;
                    //waitKey(0);
                }
            }
        }
    }
    if (cameraMatrix.empty())
    {
        return aruco::calibrateCameraCharuco(markerCornersVec, markerIdsVec, board, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);
    }
    else
        return 0;
    }
    catch( cv::Exception& e )
    {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
    return 0;
}

void remapping(vector<Mat>& images, const Mat& cameraMatrix, const Mat& distCoeffs,const Size imageSize, Mat& map1, Mat& map2){
    Mat view, rview;
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
    for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        view = *iter;
        remap(view, rview, map1, map2, INTER_LINEAR);
        //imshow("Remapped Image", rview);
        //waitKey(0);
    }
}

vector<Mat> getImages(vector<string> paths){
    vector<Mat> images;
    for (size_t i = 0; i < paths.size(); i++)
    {
        Mat img = imread(paths.at(i));
        string sub;
        size_t found = paths.at(i).find_last_of("/");
        sub = sub.substr(found+1, sub.size());
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

void NyKalibrering::on_kalibrere_clicked()
{
    ui->listWidget->selectAll();

    vector<std::string> pathVector;

    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    for(int i = 0; i < items.size(); i++)
       pathVector.push_back(path+"/"+items.at(i)->text().toStdString());

    vector<Mat> images = getImages(pathVector);

    FSClass fsCamera(path+"/cameraData.yml", localTime);
    FSClass fsRobot(path+"/robotData.yml", localTime);
    FSClass fsHandEye(path+"/handEyeData.yml", localTime);

    const Size imageSize = Size(images.at(0).rows, images.at(0).cols);
    Mat cameraMatrix, distCoeffs, map1, map2;
    vector<Mat> rvectors, tvectors;

    double repError = findArucoMarkers(images, cameraMatrix, distCoeffs, rvectors, tvectors);

    fsCamera.writeCamera(cameraMatrix, distCoeffs);

    cout << "repError = " << repError << endl;

    remapping(images, cameraMatrix, distCoeffs, imageSize, map1, map2);

    findArucoMarkers(images, cameraMatrix, distCoeffs, rvectors, tvectors);

    fstream fin;
    fin.open("/home/jeppe/Hentet/charuco/calib/robot_poses.csv", ios::in);
    vector<string> row;
    string line, word;
    vector<vector<long double>> robotPoses;
    while (robotPoses.size() < 54)
    {
        row.clear();
        vector<long double> poses;
        getline(fin, line);
        stringstream s(line);
        int rowlength =0;
        while(getline(s, word, ',')){
            row.push_back(word);
            rowlength++;
        }
        for	(int i=0; i<rowlength; i++){
            poses.push_back(stold(row.at(i)));
        }
        robotPoses.push_back(poses);
    }
    vector<Mat> rmVec;
    vector<Mat> tmVec;
    Mat cam2GripRM, cam2GripTM;
    calibrateHandEye(rmVec, tmVec, rvectors, tvectors, cam2GripRM, cam2GripTM);
    fsHandEye.writeHandEye(cam2GripRM, cam2GripTM);
}

