#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "kalibrering.h"
#include "kalibreringsFunktioner.h"
#include "analyse.h"

#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>

#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>

#include <pylon/PylonBase.h>
#include <pylon/PylonIncludes.h>
#include <pylon/TlFactory.h>
#include <pylon/ImageFormatConverter.h>
#include <pylon/PylonImage.h>

using namespace cv;
using namespace std;
using namespace ur_rtde;
using namespace Pylon;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    QFileDialog *folder = new QFileDialog;
    folder->setFileMode(QFileDialog::Directory);
    folder->setOption(QFileDialog::ShowDirsOnly);
    folder->setWindowTitle("Vælg mappe for Kalibrering");
    int result = folder->exec();
    if(result)
    {
        ui->setupUi(this);
        ui->camera_frame->setStyleSheet("background-color: rgb(255, 0, 0);");
        ui->robot_frame->setStyleSheet("background-color: rgb(255, 0, 0);");
        path = folder->selectedFiles().at(0);
    }
 /*   else
    {
        QMessageBox msg;
        msg.setText("Ingen mappe valgt, lukker program");
    }*/
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_Kalibrering_clicked()
{
    Kalibrering *kalibrering = new Kalibrering(this);
    //
    kalibrering->setWindowTitle("Kalibrering");
    kalibrering->setPath(path+"/");
    kalibrering->exec();
}

void MainWindow::on_Analyse_clicked()
{
    Analyse analyse;
    analyse.setModal(true);
    analyse.setWindowTitle("Analyse");
    std::string p = path.toStdString()+"/";
    analyse.setPath(p);
    analyse.exec();
}

void MainWindow::on_cameraButton_clicked()
{
    if(!camera)
    {
        camera = true;
        ui->camera_frame->setStyleSheet("background-color: rgb(0, 128, 0);");
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
            Mat openCvImage, grayImage;

            camera.StartGrabbing(10, GrabStrategy_LatestImageOnly);

            CGrabResultPtr ptrGrabResult;

            int wait = 0;
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
                            cvtColor(openCvImage, grayImage, COLOR_BGR2GRAY);
                            break;
                        }
                    }
                }
            }
            Mat gausmask, dst;

            double alpha = 1.5;
            double beta = -0.5;

            GaussianBlur( grayImage, gausmask, Size( 5, 5 ), 0, 0 );
            gausmask = grayImage - gausmask;
            addWeighted(grayImage, alpha, gausmask, beta, 0.0, dst);
            namedWindow("sharp",1);
            imshow("sharp", dst);

            std::vector<int> markerIds2;
            std::vector<std::vector<cv::Point2f> > markerCorners2;
            cv::aruco::detectMarkers(grayImage, board->dictionary, markerCorners2, markerIds2, params);

            std::vector<cv::Point2f> charucoCorners2;
            std::vector<int> charucoIds2;

            Mat charucoCopy;
            grayImage.copyTo(charucoCopy);
                    //cv::aruco::drawDetectedCornersCharuco(charucoCopy, charucoCorners2, charucoIds2, cv::Scalar(255,0,0));

            if (markerIds2.size() > 0)
            {
                cv::aruco::drawDetectedMarkers(charucoCopy, markerCorners2, markerIds2);
                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                cv::aruco::interpolateCornersCharuco(markerCorners2, markerIds2, grayImage, board, charucoCorners, charucoIds);
                        // if at least one charuco corner detected
                if (charucoIds.size() > 0)
                cv::aruco::drawDetectedCornersCharuco(charucoCopy, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
            }
            namedWindow("charucoMarkers", 1);
            cv::imshow("charucoMarkers", charucoCopy);
            waitKey(0);
        }

        catch (const GenericException &e)
        {
                    // Error handling
                    cerr << "An exception occurred." << endl
                        << e.GetDescription() << endl;
        }
    }
    else
    {
        camera = false;
        ui->camera_frame->setStyleSheet("background-color: rgb(255, 0, 0);");
    }

}

void MainWindow::on_robotButton_clicked()
{
    if(!robot)
    {
        robot = true;
        ui->robot_frame->setStyleSheet("background-color: rgb(0, 128, 0);");
        string hostname = "192.168.250.1";

        RTDEControlInterface rtde_control(hostname);
        RTDEReceiveInterface rtde_receive(hostname);

        vector<double> joint_positions = rtde_receive.getActualQ();
        vector<double> cartesian_positions = rtde_receive.getActualTCPPose();
        for (size_t i = 0; i < joint_positions.size(); i++)
        {
            qDebug() << joint_positions.at(i) << " ";
        }
    //joint_positions.at(0)=joint_positions.at(0)-0.5;

        joint_positions.at(0) = 0.171629;
        joint_positions.at(1) = -1.80515;
        joint_positions.at(2) = -1.44173;
        joint_positions.at(3) = -1.52207;
        joint_positions.at(4) = 1.5658;
        joint_positions.at(5) = -2.13749;

        rtde_control.moveJ(joint_positions);
    }
    else
    {
        robot = false;
        ui->robot_frame->setStyleSheet("background-color: rgb(255, 0, 0);");
    }
}
