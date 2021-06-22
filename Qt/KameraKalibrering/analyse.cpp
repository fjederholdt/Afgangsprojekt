#include "analyse.h"
#include "ui_analyse.h"
#include "fsclass.h"
#include "kalibreringsFunktioner.h"

using namespace std::filesystem;
using namespace std;
using namespace cv;
using namespace Pylon;
using namespace ur_rtde;

Analyse::Analyse(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Analyse)
{
    ui->setupUi(this);
}

Analyse::~Analyse()
{
    delete ui;
}

void Analyse::setPath(std::string &mainPath)
{
    this->path = mainPath;
    ui->tableWidget->setColumnCount(3);
    QStringList tableHeader;
    tableHeader << "Dato" << "Antal Billeder" << "Rep Error";
    ui->tableWidget->setHorizontalHeaderLabels(tableHeader);
    ui->tableWidget->horizontalHeader()->setStretchLastSection(true);
    ui->tableWidget->verticalHeader()->setVisible(false);
    ui->tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->tableWidget->setShowGrid(false);
    std::vector<std::string> pathVector;
    std::string sub, suffix;
    int maxCols = 3;
    for(const auto & entry : directory_iterator(path))
    {
        sub = entry.path();
        std::size_t found = sub.find_last_of("/");
        sub = sub.substr(found+1,sub.size());
        pathVector.push_back(sub);
    }
    ui->tableWidget->setRowCount(pathVector.size());
    std::vector<std::string> billedePath;
    std::string repErrorPath;
    fstream repin;
    for (size_t i = 0; i < pathVector.size(); i++)
    {
        for(const auto & entry : directory_iterator(path+pathVector.at(i)))
        {
            sub = entry.path();
            std::size_t found = sub.find_last_of(".");
            sub = sub.substr(found,sub.size());
            if(sub == ".png")
            {
                billedePath.push_back(sub);
            }
            else if(sub == ".txt")
            {
                repErrorPath = entry.path();
            }
        }
        for(int currentCol = 0; currentCol < maxCols; currentCol++)
        {
            switch(currentCol){
                case 0:
                {
                    QTableWidgetItem *item = new QTableWidgetItem();
                    item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                    item->setText(QString::fromStdString(pathVector.at(i)));
                    ui->tableWidget->setItem(i,0,item);
                    break;
                }
                case 1:
                {
                    QTableWidgetItem *item = new QTableWidgetItem();
                    item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                    item->setText(QString::fromStdString(std::to_string(billedePath.size())));
                    item->setTextAlignment(Qt::AlignCenter);
                    ui->tableWidget->setItem(i,1,item);
                    break;
                }
                case 2:
                {
                    if(repErrorPath.empty())
                    {
                        QTableWidgetItem *item = new QTableWidgetItem();
                        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                        item->setText(QString::fromStdString("0.00"));
                        item->setTextAlignment(Qt::AlignRight);
                        ui->tableWidget->setItem(i,2,item);
                        break;
                    }
                    else
                    {
                        repin.open(repErrorPath, ios::in);
                        string line, sub;
                        while(getline(repin, line))
                        {
                            size_t found = line.find("=");
                            sub = line.substr(found+2, line.size());
                        }
                        repin.close();
                        QTableWidgetItem *item = new QTableWidgetItem();
                        item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                        item->setText(QString::fromStdString(sub));
                        item->setTextAlignment(Qt::AlignRight);
                        ui->tableWidget->setItem(i,2,item);
                        repErrorPath.clear();
                        break;
                    }
                }
            }
        }
        billedePath.clear();
    }
    ui->tableWidget->resizeColumnsToContents();
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);

}

void Analyse::on_anvend_valgte_kalibrering_clicked()
{
    QItemSelectionModel * select = ui->tableWidget->selectionModel();
    QList<QModelIndex> rows = select->selectedRows();
    if(rows.size() == 1)
    {
        QModelIndex index = rows.at(0);
        int row = index.row();
        int col = 0;
        QString datoStr = ui->tableWidget->item(row,col)->text();
        QString kalibPath = QString::fromStdString(path)+datoStr;
        qDebug() << kalibPath;
    }
    else if(rows.size() > 1)
    {
        QMessageBox msg;
        msg.setText(QString::fromStdString("For mange kalibreringer valgt - vælg kun én kalibrering"));
        msg.exec();
    }
    else
    {
        QMessageBox msg;
        msg.setText(QString::fromStdString("Ingen kalibrering valgt - vælg kalibrering fra tabelen"));
        msg.exec();
    }
}

void Analyse::on_test_kalibrering_clicked()
{
    QItemSelectionModel * select = ui->tableWidget->selectionModel();
    QList<QModelIndex> rows = select->selectedRows();

    if(rows.size() == 1)
    {
        QModelIndex index = rows.at(0);
        int row = index.row();
        int col = 0;
        QString datoStr = ui->tableWidget->item(row,col)->text();
        string kalibPath = path+datoStr.toStdString();

        Mat grayImage grayRemap, cameraMatrix, distCoeffs, map1, map2;

        FSClass fsCamera(kalibPath+"/cameraData.yml", datoStr.toStdString());

        fsCamera.readCamera(cameraMatrix, distCoeffs);

        int cameraExposure = 60000;

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

            camera.StartGrabbing(5, GrabStrategy_LatestImageOnly);

            CGrabResultPtr ptrGrabResult;

            while(camera.IsGrabbing())
            {
                camera.RetrieveResult(2000, ptrGrabResult, TimeoutHandling_ThrowException);

                if(ptrGrabResult->GrabSucceeded())
                {
                    formatConverter.Convert(pylonImage, ptrGrabResult);
                    openCvImage = Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t*)pylonImage.GetBuffer());

                    cvtColor(openCvImage, grayImage, COLOR_BGR2GRAY);
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
        vector<Mat> grayVector;
        grayVector.push_back(grayImage);
        Size imageSize = Size(grayImage.rows, grayImage.cols);
        remapping(grayVector, cameraMatrix, distCoeffs, imageSize, map1, map2);
        remap(grayImage, grayRemap, map1, map2, INTER_LINEAR);

        RTDEReceiveInterface rtde_receive(hostname);
        std::vector<double> joint_positions = rtde_receive.getActualQ();


    }
    else if(rows.size() > 1)
    {
        QMessageBox msg;
        msg.setText(QString::fromStdString("For mange kalibreringer valgt - vælg kun én kalibrering"));
        msg.exec();
    }
    else
    {
        QMessageBox msg;
        msg.setText(QString::fromStdString("Ingen kalibrering valgt - vælg kalibrering fra tabelen"));
        msg.exec();
    }


}

pcl::visualization::PCLVisualizer::Ptr viz (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud){
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1);
    viewer->initCameraParameters ();
    return (viewer);
}

void Analyse::on_visualiser_clicked()
{
    QItemSelectionModel * select = ui->tableWidget->selectionModel();
    QList<QModelIndex> rows = select->selectedRows();

    ui->imageLabel->setBackgroundRole(QPalette::Base);
    ui->imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);

    std::string repErrorPath;
    fstream repin;

    if(rows.size() == 1)
    {
        QModelIndex index = rows.at(0);
        int row = index.row();
        QString datoStr = ui->tableWidget->item(row,0)->text();
        QString billeder = ui->tableWidget->item(row,1)->text();
        std::string kalibPath = path+datoStr.toStdString();
        string sub;
        for(const auto & entry : directory_iterator(kalibPath))
        {
            sub = entry.path();
            std::size_t found = sub.find_last_of(".");
            sub = sub.substr(found,sub.size());
            if(sub == ".txt")
            {
                repErrorPath = entry.path();
            }
        }

        repin.open(repErrorPath, ios::in);
        string line;
        vector<double> repErrors;
        while(getline(repin, line))
        {
            if(line != "repErrors: ")
            {
                size_t found = line.find_first_of("=");
                sub = line.substr(found+2, line.size());
                repErrors.push_back(stod(sub));
            }
        }
        repin.close();

        double max = *max_element(repErrors.begin(), repErrors.end());
        int image_nr;
        for(size_t i = 0; i < repErrors.size(); i++)
        {
            if(max == repErrors.at(i))
                image_nr = i;
        }

        int histSize = repErrors.size();

        int hist_w = 600, hist_h = 400;
        int bin_w = cvRound( (double) hist_w/histSize );
        Mat histImage( hist_h, hist_w, CV_8UC1);
        //normalize(grayHist, grayHist, 0, histImage.rows, NORM_MINMAX, -1, Mat() );

        for( int i = 0; i < histSize; i++ )
        {
            cv::rectangle( histImage, Point( bin_w*(i), cvRound(hist_h) ),
                Point( bin_w*(i), hist_h - cvRound(repErrors.at(i)*500) ),
                Scalar(255,0,0), 2, 8, 0  );
        }
        imwrite(kalibPath+"/histogram.jpg", histImage);

        namedWindow("test", 1);
        imshow("test", histImage);

        QPixmap hist(QString::fromStdString(kalibPath+"/histogram.jpg"));
        ui->imageLabel->setPixmap(hist);
        ui->imageLabel->setScaledContents(true);
        ui->maxRep->setText(QString::fromStdString("Max reperror: "+to_string(image_nr)+".png: "+to_string(max)));
        ui->maxRep->setScaledContents(true);

        FSClass fsCamera(kalibPath+"/cameraData.yml", datoStr.toStdString());
        FSClass fsRobot(kalibPath+"/robotData.yml", datoStr.toStdString());
        FSClass fsHandEye(kalibPath+"/handEyeData.yml", datoStr.toStdString());

        Mat cameraRm, cameraTm;
        Mat robotRm, robotTm;
        Mat handEyeRm, handEyeTm;

        fsCamera.readCamera(cameraRm, cameraTm);
        fsRobot.readRobot(robotRm, robotTm);
        fsHandEye.readHandEye(handEyeRm, handEyeTm);


      /*  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointXYZ robot;
        pcl::PointXYZ camera;
        pcl::PointXYZ handeye;

        camera._PointXYZ::x = cameraTm.at<double>(0);
        camera._PointXYZ::y = cameraTm.at<double>(1);
        camera._PointXYZ::z = cameraTm.at<double>(2);

        robot._PointXYZ::x = robotTm.at<double>(0);
        robot._PointXYZ::y = robotTm.at<double>(1);
        robot._PointXYZ::z = robotTm.at<double>(2);

        handeye._PointXYZ::x = handEyeTm.at<double>(0);
        handeye._PointXYZ::y = handEyeTm.at<double>(1);
        handeye._PointXYZ::z = handEyeTm.at<double>(2);

        basic_cloud_ptr->push_back(camera);
        basic_cloud_ptr->push_back(robot);
        basic_cloud_ptr->push_back(handeye);

        basic_cloud_ptr->width = basic_cloud_ptr->size();
        basic_cloud_ptr->height = 1;

        pcl::visualization::PCLVisualizer::Ptr viewer;

        viewer = viz((pcl::PointCloud<pcl::PointXYZ>::ConstPtr)basic_cloud_ptr);

        while(!viewer->wasStopped())
        {
            viewer->spinOnce(100);
        }
        //qDebug() << kalibPath;*/
    }
    else if(rows.size() > 1)
    {
        QMessageBox msg;
        msg.setText(QString::fromStdString("For mange kalibreringer valgt - vælg kun én kalibrering"));
        msg.exec();
    }
    else
    {
        QMessageBox msg;
        msg.setText(QString::fromStdString("Ingen kalibrering valgt - vælg kalibrering fra tabelen"));
        msg.exec();
    }

}

void Analyse::on_annuller_clicked()
{
    Analyse::close();
}

