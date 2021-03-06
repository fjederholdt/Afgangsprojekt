#include "nykalibrering.h"
#include "ui_nykalibrering.h"
#include "kalibrering.h"
#include "kalibreringsFunktioner.h"
#include "kameraFunktioner.h"
#include "dhparams.h"

using namespace std::filesystem;
using namespace std;
using namespace cv;
using namespace ur_rtde;
using namespace Pylon;

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
    RTDEReceiveInterface rtde_receive(hostname);
    vector<string> paths;
    vector<Mat> images;
    vector<vector<double>> robotPoses;
    string imagenr;

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

        /*GenApi::CEnumerationPtr exposureAuto( nodemap.GetNode( "ExposureAuto"));
        if ( GenApi::IsWritable( exposureAuto))
        {
            exposureAuto->FromString("Off");
        }

        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        if(exposureTime.IsValid())
        {
            if(cameraExposure >= exposureTime->GetMin() && cameraExposure <= exposureTime->GetMax())
            {
                exposureTime->SetValue(cameraExposure);
            }
            else
            {
                exposureTime->SetValue(exposureTime->GetMin());
            }
        }
        else
        {
            std::cout << ">> Failed to set exposure value." << std::endl;
        }
        */

        CGrabResultPtr ptrGrabResult;

        camera.StartGrabbing(GrabStrategy_LatestImageOnly);

        namedWindow("Camera feed", 1);
        bool feeding = true;
        while(feeding)
        {
            while(camera.IsGrabbing())
            {
                camera.RetrieveResult(1100, ptrGrabResult, TimeoutHandling_ThrowException);

                if(ptrGrabResult->GrabSucceeded())
                {
                    formatConverter.Convert(pylonImage, ptrGrabResult);
                    openCvImage = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
                }


            Mat grayImage;
            cvtColor(openCvImage, grayImage, COLOR_BGR2GRAY);
            imshow("Camera feed", grayImage);

            int wait = waitKey(1);
            if (wait == 97)
            {
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

                images.push_back(grayImage);
                paths.push_back(path+"/"+imagenr);
                robotPoses.push_back(rtde_receive.getActualQ());
                image_nr++;
            }
            else if (wait == 27)
            {
                destroyAllWindows();
                feeding = false;
                break;
            }
            }
        }
    }
    catch (const GenericException &e)
    {
        //Error handling
        cerr << "An exception occurred." << endl
            << e.GetDescription() << endl;
    }

    for (size_t i = 0; i < images.size(); i++)
    {
        imwrite(paths.at(i), images.at(i));
    }

    for(size_t i = 0; i < robotPoses.size(); i++)
    {
        if(i < 10)
        {
            imagenr = "00"+to_string(i)+".png";
        }
        else if(i > 9 && i < 100)
        {
            imagenr = "0"+to_string(i)+".png";
        }
        else
            imagenr = to_string(i)+".png";


        output_file << imagenr+",robot:,";
        std::ostream_iterator<double> output_iterator(output_file, ",");
        std::copy(robotPoses.at(i).begin(), robotPoses.at(i).end(), output_iterator);
        output_file << endl;
        addList(imagenr);
    }
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

void NyKalibrering::on_kalibrere_clicked()
{
    output_file.close();
    vector<std::string> billeder;
    ui->listWidget->selectAll();
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    if(!items.empty())
    {
        try
        {
            for(int i = 0; i < items.size(); i++)
            {
                billeder.push_back(path+"/"+items.at(i)->text().toStdString());
            }

            fstream fin;
            fin.open(path+"/robotPoses.csv", ios::in);
            vector<string> row;
            string line, word;
            vector<vector<double>> robotPoses;
            while (robotPoses.size() < billeder.size())
            {
                row.clear();
                vector<double> pose;
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
                        {
                            row.push_back(word);
                        }
                    }
                }
                if(!row.empty())
                {
                    for	(size_t i = 0; i < row.size(); i++)
                    {
                        pose.push_back(stod(row.at(i)));
                    }
                    robotPoses.push_back(pose);
                }
            }

            vector<Mat> images = getImages(billeder);

            FSClass fsCamera(path+"/cameraData.yml", localTime);
            FSClass fsRobot(path+"/robotData.yml", localTime);
            FSClass fsHandEye(path+"/handEyeData.yml", localTime);

            Mat cameraMatrix, distCoeffs, map1, map2;
            vector<Mat> rvectors, tvectors, rotationMat, translationMat;
            vector<vector<int>> charucoIds;
            vector<vector<Point2f>> charucoCorners;

            vector<double> repErrors = calibrateCharuco(images, cameraMatrix, distCoeffs, charucoCorners, charucoIds, rotationMat, translationMat);
                    //findArucoMarkers(images, cameraMatrix, distCoeffs, rvectors, tvectors);

            fsCamera.writeCamera(cameraMatrix, distCoeffs, rotationMat.at(0), translationMat.at(0));

            ofstream repErr;
            repErr.open(path+"/repError.txt");
            repErr << "repErrors: " << endl;
            for(size_t i = 0; i < repErrors.size(); i++)
            {
                repErr << "image number: " << i << " = " << repErrors.at(i) << endl;
            }

            repErr.close();


            vector<Mat> rview;
            remapping(images, cameraMatrix, distCoeffs, map1, map2, rview);



            charucoBoardPose(images, cameraMatrix, distCoeffs, charucoCorners, charucoIds, rvectors, tvectors);
            //findArucoMarkers(images, cameraMatrix, distCoeffs, rvectors, tvectors);

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
            calibrateHandEye(rmVec, tmVec, rotationMat, translationMat, cam2GripRM, cam2GripTM);
            fsHandEye.writeHandEye(cam2GripRM, cam2GripTM);

            QMessageBox msg;
            msg.setText("Kalibrering er færdig");
            int retur = msg.exec();

            if(retur == QMessageBox::Ok)
                NyKalibrering::close();
        }
        catch (Exception& e)
        {
            qDebug() << "Exception thrown: " << e.what();
        }

    }
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

        /*
        GenApi::CEnumerationPtr exposureAuto( nodemap.GetNode( "ExposureAuto"));
        if ( GenApi::IsWritable( exposureAuto))
        {
            exposureAuto->FromString("Off");
        }

        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        if(exposureTime.IsValid())
        {
            if(cameraExposure >= exposureTime->GetMin() && cameraExposure <= exposureTime->GetMax())
            {
                exposureTime->SetValue(cameraExposure);
            }
            else
            {
                exposureTime->SetValue(exposureTime->GetMin());
            }
        }
        else
        {
            std::cout << ">> Failed to set exposure value." << std::endl;
        }
        */

        camera.StartGrabbing(10, GrabStrategy_LatestImageOnly);

        CGrabResultPtr ptrGrabResult;

        while(camera.IsGrabbing())
        {
            camera.RetrieveResult(1000, ptrGrabResult, TimeoutHandling_ThrowException);

            if(ptrGrabResult->GrabSucceeded())
            {
                formatConverter.Convert(pylonImage, ptrGrabResult);
                openCvImage = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());

                Mat grayImage;
                cvtColor(openCvImage, grayImage, COLOR_BGR2GRAY);

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
    std::vector<double> start_joint_positions = rtde_receive.getTargetQ();
    std::vector<double> cartesian_positions = rtde_receive.getActualTCPPose();

    unsigned int microsecond = 1000000;
    tagBillede();
    cartesian_positions.at(0) += 0.05;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    usleep(3 * microsecond);
    tagBillede();
    cartesian_positions.at(1) += 0.05;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    usleep(3 * microsecond);
    tagBillede();
    cartesian_positions.at(0) -= 0.05;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    usleep(3 * microsecond);
    tagBillede();
    cartesian_positions.at(0) -= 0.05;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    usleep(3 * microsecond);
    tagBillede();
    cartesian_positions.at(1) -= 0.05;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    usleep(3 * microsecond);
    tagBillede();
    cartesian_positions.at(1) -= 0.05;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    usleep(3 * microsecond);
    tagBillede();
    cartesian_positions.at(0) += 0.05;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    usleep(3 * microsecond);
    tagBillede();
    cartesian_positions.at(0) += 0.05;
    rtde_control.moveL(cartesian_positions,0.25,1.2,false);
    usleep(3 * microsecond);
    tagBillede();
    rtde_control.moveJ(start_joint_positions);
}


void NyKalibrering::on_Charucomarkers_clicked()
{
    vector<std::string> billeder;
    ui->listWidget->selectAll();
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    Mat grayImage;

    if (!items.empty())
    {
        try
        {
            for(int i = 0; i < items.size(); i++)
            {
                billeder.push_back(path+"/"+items.at(i)->text().toStdString());
            }
            vector<Mat> grayImages = getImages(billeder);


            std::vector<int> markerIds2;
            std::vector<std::vector<cv::Point2f> > markerCorners2;

            std::vector<cv::Point2f> charucoCorners2;
            std::vector<int> charucoIds2;


            for(vector<Mat>::iterator iter = grayImages.begin(); iter != grayImages.end(); iter++)
            {
                Mat inputImage = *iter;
                cv::aruco::detectMarkers(inputImage, board->dictionary, markerCorners2, markerIds2, params);
                if (markerIds2.size() > 0)
                {
                    cv::aruco::drawDetectedMarkers(inputImage, markerCorners2, markerIds2, cv::Scalar(255,0,0));
                    std::vector<cv::Point2f> charucoCorners;
                    std::vector<int> charucoIds;
                    cv::aruco::interpolateCornersCharuco(markerCorners2, markerIds2, grayImages, board, charucoCorners, charucoIds);
                            // if at least one charuco corner detected
                    if (charucoIds.size() > 0){
                        cv::aruco::drawDetectedCornersCharuco(inputImage, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
                    }
                }
                namedWindow("charucoMarkers", 1);
                cv::imshow("charucoMarkers", inputImage);
                waitKey(0);

            }
        }
    catch (Exception& e){}
    }
}

