#include "nykalibrering.h"
#include "ui_nykalibrering.h"
#include "kalibrering.h"
#include "dhparams.h"

using namespace std::filesystem;
using namespace std;
using namespace cv;
using namespace ur_rtde;
using namespace Pylon;

const float chessSquareDim = 0.02f;
const float arucoSquareDim = 0.015f;
const Size chessboardDim = Size(9, 14);

NyKalibrering::NyKalibrering(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::NyKalibrering)
{
    ui->setupUi(this);
}

NyKalibrering::~NyKalibrering()
{
    delete ui;
    output_file.close();
}

void NyKalibrering::setPath(std::string &kalibpath)
{
    time_t rawtime;
    time(&rawtime);
    localTime = asctime(localtime(&rawtime));
    localTime.pop_back();
    this->path = kalibpath+localTime;
    mode_t mode = 0755;
    char *copypath = strdup(path.c_str());
    if(mkdir(copypath, mode) != 0 && errno != EEXIST)
    {
        std::cerr << "Error : " << strerror(errno) << std::endl;
    }
    output_file.open(path+"/robotPoses.csv");
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
    Pylon::PylonAutoInitTerm autoInitTerm;
    bool saveImage = true;
    try
    {
        CInstantCamera camera(CTlFactory::GetInstance().CreateFirstDevice());

        GenApi::INodeMap& nodemap = camera.GetNodeMap();
        camera.Open();
        GenApi::CIntegerPtr width = nodemap.GetNode("Width");
        GenApi::CIntegerPtr height = nodemap.GetNode("Height");

        camera.MaxNumBuffer = 5;

        CImageFormatConverter formatConverter;
        formatConverter.OutputPixelFormat = PixelType_BGR8packed;

        CPylonImage pylonImage;
        Mat openCvImage;

        camera.StartGrabbing(10, GrabStrategy_LatestImageOnly);

        CGrabResultPtr ptrGrabResult;

        while(camera.IsGrabbing())
        {
            camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

            if(ptrGrabResult->GrabSucceeded())
            {
                formatConverter.Convert(pylonImage, ptrGrabResult);
                openCvImage = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());

                if(saveImage)
                {
                    Mat grayImage;
                    cvtColor(openCvImage, grayImage, COLOR_BGR2GRAY);
                    imwrite(path+"/"+to_string(image_nr)+".png", grayImage);
                }
            }
            /*namedWindow("Camera feed", 1);
            imshow("Camera feed", openCvImage);
            if (waitKey(1) >= 0)
            {
                destroyAllWindows();
                break;
            }*/
        }
    }
    catch (const GenericException &e)
    {
                // Error handling
                cerr << "An exception occurred." << endl
                    << e.GetDescription() << endl;
    }

    RTDEReceiveInterface rtde_receive(hostname);
    std::vector<double> joint_positions = rtde_receive.getActualQ();
    output_file << to_string(image_nr)+".png,robot:,";
    std::ostream_iterator<double> output_iterator(output_file, ",");
    std::copy(joint_positions.begin(), joint_positions.end(), output_iterator);
    output_file << endl;

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
                    Rodrigues(rvec, rMat);
                    Rodrigues(tvec, tMat);
                    rvectors.push_back(rMat);
                    tvectors.push_back(tMat);
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
    {
        pathVector.push_back(path+"/"+items.at(i)->text().toStdString());
    }

    fstream fin;
    fin.open(path+"/robotPoses.csv", ios::in);
    vector<string> row;
    string line, word;
    vector<vector<long double>> robotPoses;
    while (robotPoses.size() < pathVector.size())
    {
        row.clear();
        vector<long double> pose;
        getline(fin, line);
        stringstream s(line);
        int rowlength = 0;
        while(getline(s, word, ',')){
            for(int i = 0; i < items.size(); i++)
            {
                if(word.find(".png"))
                {
                    row.push_back(word);
                }
                if(word.find(".png") == std::string::npos || word.find("robot:") == std::string::npos)
                {
                    row.push_back(word);
                    rowlength++;
                }
                else
                    continue;
            }

        }
        for	(int i=0; i<rowlength; i++){
            pose.push_back(stold(row.at(i)));
        }
        robotPoses.push_back(pose);
    }

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

    vector<Mat> rmVec;
    vector<Mat> tmVec;
    for (size_t i = 0; i < robotPoses.size(); i++)
    {
        DHParams dhp(robotPoses.at(i));
        dhp.calculateDH();
        rmVec.push_back(dhp.getRM());
        tmVec.push_back(dhp.getTM());
    }

    Mat cam2GripRM, cam2GripTM;
    calibrateHandEye(rmVec, tmVec, rvectors, tvectors, cam2GripRM, cam2GripTM);
    fsHandEye.writeHandEye(cam2GripRM, cam2GripTM);

    QMessageBox msg;
    msg.setText("Kalibrering er færdig");
    int retur = msg.exec();

    if(retur == QMessageBox::Ok)
        NyKalibrering::close();
}

void NyKalibrering::on_annuller_clicked()
{
    QMessageBox msg;
    msg.setText(QString::fromStdString("ADVARSEL: Sletter mappe: Kalibreringer/"+localTime));
    msg.setInformativeText("Vil du fortsætte?");
    msg.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
    int retur = msg.exec();

    if(retur == QMessageBox::Ok)
    {
        boost::filesystem::remove_all(path);
        NyKalibrering::close();
    }
}

