#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "kalibrering.h"
#include "analyse.h"

#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

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
            /*
            double minLowerLimit = camera.AutoExposureTimeLowerLimitRaw.GetMin();
            double maxUpperLimit = camera.AutoExposureTimeUpperLimitRaw.GetMax();
            camera.AutoExposureTimeLowerLimitRaw.SetValue(minLowerLimit);
            camera.AutoExposureTimeUpperLimitRaw.SetValue(maxUpperLimit);
            // Set the target brightness value to 128
            camera.AutoTargetValue.SetValue(128);
            // Select auto function ROI 1
            camera.AutoFunctionAOISelector.SetValue(AutoFunctionAOISelector_AOI1);
            // Enable the 'Intensity' auto function (Gain Auto + Exposure Auto)
            // for the auto function ROI selected
            camera.AutoFunctionAOIUsageIntensity.SetValue(true);
            // Enable Exposure Auto by setting the operating mode to Continuous
            camera.ExposureAuto.SetValue(ExposureAuto_Continuous);
            */
            GenApi::INodeMap& nodemap = camera.GetNodeMap();
            camera.Open();
            GenApi::CIntegerPtr width = nodemap.GetNode("Width");
            GenApi::CIntegerPtr height = nodemap.GetNode("Height");

            camera.MaxNumBuffer = 5;

            CImageFormatConverter formatConverter;
            formatConverter.OutputPixelFormat = PixelType_BGR8packed;

            CPylonImage pylonImage;
            Mat openCvImage;

            Size frameSize = Size((int)width->GetValue(), (int)height->GetValue());

            camera.StartGrabbing(10, GrabStrategy_LatestImageOnly);

            CGrabResultPtr ptrGrabResult;
            namedWindow("Camera feed", 1);
            while(camera.IsGrabbing())
            {
                camera.RetrieveResult(5000, ptrGrabResult, TimeoutHandling_ThrowException);

                if(ptrGrabResult->GrabSucceeded())
                {
                    formatConverter.Convert(pylonImage, ptrGrabResult);
                    openCvImage = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
                }
                imshow("Camera feed", openCvImage);
            }
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
        for (size_t i = 0; i < joint_positions.size(); i++)
        {
            qDebug() << joint_positions.at(i) << " ";
        }
    /*joint_positions.at(0)=joint_positions.at(0)-0.5;

    rtde_control.moveJ(joint_positions);*/
    }
    else
    {
        robot = false;
        ui->robot_frame->setStyleSheet("background-color: rgb(255, 0, 0);");
    }
}
