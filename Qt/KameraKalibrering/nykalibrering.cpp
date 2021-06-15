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
    ui->listWidget->setSelectionMode(QAbstractItemView::MultiSelection);
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
    string imagenr;
    if(image_nr < 10)
    {
        imagenr = "00"+to_string(image_nr)+".png";
    }
    else if(image_nr > 9 && image_nr < 100)
    {
        imagenr = "0"+to_string(image_nr)+".png";
    }
    else
        imagenr = to_string(image_nr)+".png";

    Pylon::PylonAutoInitTerm autoInitTerm;
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

        camera.StartGrabbing(100, GrabStrategy_LatestImageOnly);

        CGrabResultPtr ptrGrabResult;
        int wait=0;
        while(wait == 0)
        {
            while(camera.IsGrabbing())
            {
                camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

                if(ptrGrabResult->GrabSucceeded())
                {
                    formatConverter.Convert(pylonImage, ptrGrabResult);
                    openCvImage = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());

                    namedWindow("Camera feed", 1);
                    imshow("Camera feed", openCvImage);
                    wait = waitKey(0);
                    if (wait)
                    {
                        Mat grayImage, gausmask, dst;
                        cvtColor(openCvImage, grayImage, COLOR_BGR2GRAY);
                        //double alpha = 0.5;
                        //double beta = (1.0-alpha);
                        //gausmask = grayImage.clone();
                        //GaussianBlur( grayImage, gausmask, Size( 9, 9 ), 0, 0 );
                        //gausmask -= grayImage;
                        //addWeighted(grayImage, alpha, gausmask, beta, 0.0, dst);
                        imwrite(path+"/"+imagenr, grayImage);
                        destroyAllWindows();
                        break;
                    }
                }
            }
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
    output_file << imagenr+",robot:,";
    std::ostream_iterator<double> output_iterator(output_file, ",");
    std::copy(joint_positions.begin(), joint_positions.end(), output_iterator);
    output_file << endl;

    addList(imagenr);
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
    initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_8UC1, map1, map2);
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

void NyKalibrering::on_kalibrere_clicked()
{
    output_file.close();
    vector<std::string> billeder;
    ui->listWidget->selectAll();
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    for(int i = 0; i < items.size(); i++)
    {
        billeder.push_back(path+"/"+items.at(i)->text().toStdString());
    }

    fstream fin;
    fin.open(path+"/robotPoses.csv", ios::in);
    vector<string> row;
    string line, word;
    vector<vector<long double>> robotPoses;
    while (robotPoses.size() < billeder.size())
    {
        row.clear();
        vector<long double> pose;
        getline(fin, line);
        stringstream s(line);
        string png;
        bool pngFound = false;
        while(getline(s, word, ','))
        {
            if (!pngFound)
            {
                size_t found = word.find(".png");
                if(found != string::npos)
                {
                    png = path+"/"+word;
                    for(size_t i = 0; i < billeder.size(); i++)
                    {
                        if(png == billeder.at(i))
                        {
                            pngFound = true;
                            break;
                        }
                    }
                }
            }
            size_t foundPng = word.find(".png");
            size_t foundRobot = word.find("robot");
            if(foundPng == std::string::npos && foundRobot == std::string::npos)
            {
                if(pngFound)
                    row.push_back(word);
            }
        }
        if(!row.empty())
        {
            for	(size_t i = 0; i < row.size(); i++)
            {
                pose.push_back(stold(row.at(i)));
            }
            robotPoses.push_back(pose);
        }
    }

    vector<Mat> images = getImages(billeder);

    FSClass fsCamera(path+"/cameraData.yml", localTime);
    FSClass fsRobot(path+"/robotData.yml", localTime);
    FSClass fsHandEye(path+"/handEyeData.yml", localTime);

    const Size imageSize = Size(images.at(0).rows, images.at(0).cols);
    Mat cameraMatrix, distCoeffs, map1, map2;
    vector<Mat> rvectors, tvectors;

    double repError = findArucoMarkers(images, cameraMatrix, distCoeffs, rvectors, tvectors);

    fsCamera.writeCamera(cameraMatrix, distCoeffs);

    ofstream repErr;
    repErr.open(path+"/repError.txt");
    repErr << "repError: " << repError << endl;

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

void NyKalibrering::tagBillede()
{
    string imagenr;
    if(image_nr < 10)
    {
        imagenr = "00"+to_string(image_nr)+".png";
    }
    else if(image_nr > 9 && image_nr < 100)
    {
        imagenr = "0"+to_string(image_nr)+".png";
    }
    else
        imagenr = to_string(image_nr)+".png";

    Pylon::PylonAutoInitTerm autoInitTerm;
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

                Mat grayImage, gausmask, dst;
                cvtColor(openCvImage, grayImage, COLOR_BGR2GRAY);

               /* double alpha = 1.5;
                double beta = -0.5;

                GaussianBlur( grayImage, gausmask, Size( 9, 9 ), 0, 0 );
                gausmask = grayImage - gausmask;
                addWeighted(grayImage, alpha, gausmask, beta, 0.0, dst);
*/
                imwrite(path+"/"+imagenr, grayImage);

                break;
            }
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
    output_file << imagenr+",robot:,";
    std::ostream_iterator<double> output_iterator(output_file, ",");
    std::copy(joint_positions.begin(), joint_positions.end(), output_iterator);
    output_file << endl;

    addList(imagenr);
    image_nr++;
}


void NyKalibrering::on_auto_billede_clicked()
{
    RTDEControlInterface rtde_control(hostname);
    RTDEReceiveInterface rtde_receive(hostname);
    std::vector<double> start_joint_positions;
    start_joint_positions.push_back(0.105032);
    start_joint_positions.push_back(-2.43353);
    start_joint_positions.push_back(-1.9662);
    start_joint_positions.push_back(-0.28989);
    start_joint_positions.push_back(1.56231);
    start_joint_positions.push_back(-0.667773);
    rtde_control.moveJ(start_joint_positions);

    std::vector<double> cartesian_positions = rtde_receive.getActualTCPPose();

    tagBillede();
    cartesian_positions.at(0) += 0.1;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    tagBillede();
    std::vector<double> joint_positions = rtde_receive.getActualQ();
    joint_positions.at(5) += 1.57;
    rtde_control.moveJ(joint_positions);
    tagBillede();
    cartesian_positions.at(1) += 0.1;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    tagBillede();
    joint_positions = rtde_receive.getActualQ();
    joint_positions.at(5) -= 1.57;
    rtde_control.moveJ(joint_positions);
    tagBillede();
    cartesian_positions.at(0) -= 0.1;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    tagBillede();
    joint_positions = rtde_receive.getActualQ();
    joint_positions.at(5) += 1.57;
    rtde_control.moveJ(joint_positions);
    tagBillede();
    cartesian_positions.at(0) -= 0.1;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    tagBillede();
    joint_positions = rtde_receive.getActualQ();
    joint_positions.at(5) -= 1.57;
    rtde_control.moveJ(joint_positions);
    tagBillede();
    cartesian_positions.at(1) -= 0.1;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    tagBillede();
    joint_positions = rtde_receive.getActualQ();
    joint_positions.at(5) += 1.57;
    rtde_control.moveJ(joint_positions);
    tagBillede();
    cartesian_positions.at(1) -= 0.1;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    tagBillede();
    joint_positions = rtde_receive.getActualQ();
    joint_positions.at(5) -= 1.57;
    rtde_control.moveJ(joint_positions);
    tagBillede();
    cartesian_positions.at(0) += 0.1;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    tagBillede();
    joint_positions = rtde_receive.getActualQ();
    joint_positions.at(5) += 1.57;
    rtde_control.moveJ(joint_positions);
    tagBillede();
    cartesian_positions.at(0) += 0.1;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    tagBillede();
    joint_positions = rtde_receive.getActualQ();
    joint_positions.at(5) -= 1.57;
    rtde_control.moveJ(joint_positions);
    tagBillede();
    rtde_control.moveJ(start_joint_positions);

}

