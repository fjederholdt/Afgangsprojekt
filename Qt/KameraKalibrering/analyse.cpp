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
    vector<string> billedePathInit;
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
                billedePathInit.push_back(sub);
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
                    item->setText(QString::fromStdString(std::to_string(billedePathInit.size())));
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
        billedePathInit.clear();
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

        Mat grayImage, cameraMatrix, distCoeffs, map1, map2;

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

        vector<Mat> grayVector, grayRemap;
        grayVector.push_back(grayImage);
        remapping(grayVector, cameraMatrix, distCoeffs, map1, map2, grayRemap);

        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::aruco::detectMarkers(grayRemap.at(0), board->dictionary, markerCorners, markerIds, params);

        std::vector<cv::Point2f> charucoCorners;
        std::vector<int> charucoIds;

        Mat charucoCopy;
        grayRemap.at(0).copyTo(charucoCopy);
                //cv::aruco::drawDetectedCornersCharuco(charucoCopy, charucoCorners2, charucoIds2, cv::Scalar(255,0,0));

        if (markerIds.size() > 0)
        {
            cv::aruco::drawDetectedMarkers(charucoCopy, markerCorners, markerIds);
            std::vector<cv::Point2f> charucoCorners;
            std::vector<int> charucoIds;
            cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, grayImage, board, charucoCorners, charucoIds);
        }

        //aruco::getBoardObjectAndImagePoints()

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

        Mat cameraMatrix, distCoeffs;
        Mat robotRm, robotTm;
        Mat handEyeRm, handEyeTm;

        fsCamera.readCamera(cameraMatrix, distCoeffs);
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
        //qDebug() << kalibPath;



#include <pcl/visualization/pcl_visualizer.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr myCloud(new PointCloud<pcl::PointXYZ>);
pcl::PointXYZ p1(0.3,0.4,0.3);
pcl::PointXYZ p2(0.5,0.5,0.5);
myCloud.push_back(p1);
myCloud.push_back(p2);

pcl::visualization::PCLVisualizer myVis("someName");
myVis.addPointCloud(myCloud);
myVis.addCoordinateSystem(1);
myVis.setBackgroundColor(255,255,255);
myvVis.spin();
*/
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


void Analyse::on_hvis_charuco_clicked()
{
    //param.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX     Kan bruges til at forbedre præcisionen af marker corners og id. Kan være nødvendig får os.

    QItemSelectionModel * select = ui->tableWidget->selectionModel();
    QList<QModelIndex> rows = select->selectedRows();
    if(rows.size() == 1)
    {
        QModelIndex index = rows.at(0);
        int row = index.row();
        int col = 0;
        QString datoStr = ui->tableWidget->item(row,col)->text();
        string kalibPath = path+datoStr.toStdString();
        string sub;
        vector<string> billedePath;

        for(const auto & entry : directory_iterator(kalibPath))
        {
            sub = entry.path();
            std::size_t found = sub.find_last_of(".");
            sub = sub.substr(found,sub.size());
            if(sub == ".png")
            {
                billedePath.push_back(entry.path());
            }
        }

        vector<Mat> grayImages;

        FSClass fsCamera(kalibPath+"/cameraData.yml", datoStr.toStdString());
        Mat cameraMatrix, distCoeffs;
        fsCamera.readCamera(cameraMatrix, distCoeffs);

        grayImages = getImages(billedePath);

        Mat map1, map2;
        vector<Mat> rview;
        cv::Point2f cornerDistThresh(140.0,140.0);

        cv::Point2f p3;
        cv::Point2f p4;
        cv::Point2f p5;
        cv::Point2f p6;
        cv::Point2f p7;
        cv::Point2f midP;
        cv::Point2f rotatedVec;
        cv::Point2f sharpVec;
        double pixVecx = 0;
        double pixVecy = 0;
        double pixelLength = 0;
        double pMidx = 0;
        double pMidy = 0;
        Mat temp;
        float pixToMM = 0;


        remapping(grayImages, cameraMatrix, distCoeffs, map1, map2, rview);

        if (!grayImages.empty())
        {
            try
            {
                std::vector<int> markerIds;
                std::vector<std::vector<cv::Point2f> > markerCorners;

                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;
                int markcount = 0;

                for(vector<Mat>::iterator iter = rview.begin(); iter != rview.end(); iter++)
                {
                    Mat inputImage = *iter;
                    temp = inputImage;
                    markcount++;
                    cv::aruco::detectMarkers(inputImage, board->dictionary, markerCorners, markerIds, params);
                    cout << "Billed nr: " << markcount << " med antal CharucoIDs: " << markerIds.size() << endl;
                    if (markerIds.size() > 0)
                    {
                        cv::aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds, cv::Scalar(255,0,0));
                        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, inputImage, board, charucoCorners, charucoIds);
                        // if at least one charuco corner detected
                        if (charucoIds.size() > 0)
                        {
                            cv::aruco::drawDetectedCornersCharuco(inputImage, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
                        }
                    }
                    for (int i=0; i<charucoIds.size(); i++)
                    {
                        if (charucoIds.at(0) == i)
                        {
                            if (charucoIds.at(1) == charucoIds.at(0) + 1 )
                            {
                                cout << "Første hjørne koordinator: " << charucoCorners.at(0) << endl;
                                cout << "Andet hjørne koordinator: "  << charucoCorners.at(1) << endl;


                                if (fdim(charucoCorners.at(0).x , charucoCorners.at(1).x) < cornerDistThresh.x)
                                {
                                    if (fdim(charucoCorners.at(0).y , charucoCorners.at(1).y) < cornerDistThresh.y)
                                    {
                                        cv::circle(inputImage, charucoCorners.at(0), 8, cv::Scalar(55,0,255), -2);
                                        cv::circle(inputImage, charucoCorners.at(1), 8, cv::Scalar(55,0,255), -2);
                                        cv::line(inputImage, charucoCorners.at(0), charucoCorners.at(1), cv::Scalar(0,0,255), 4);
                                    }

                                    //get the vector and calculate pythagoras to calculated pixels which are not x or y dominante faced.
                                    pixVecx = charucoCorners.at(1).x - charucoCorners.at(0).x;
                                    pixVecy = charucoCorners.at(1).y - charucoCorners.at(0).y;




                                    p5.x = pixVecx;
                                    p5.y = pixVecy;
                                    pixelLength = sqrt(pow(pixVecx,2) + pow(pixVecy,2));
                                    cout << "Final Pixel Length: " << pixelLength << endl;
                                    pixToMM = float(20.0/pixelLength);
                                    cout << "Pixels per MM: " << pixToMM << endl;




                                    pMidx = (charucoCorners.at(0).x + charucoCorners.at(1).x)/2;
                                    pMidy = (charucoCorners.at(0).y + charucoCorners.at(1).y)/2;
                                    midP.x = pMidx;
                                    midP.y = pMidy;
                                    cout << "X value for midt: " << midP.x << " Y valude for midt: " << midP.y << endl;

                                    rotatedVec.x = charucoCorners.at(1).x - midP.x;
                                    rotatedVec.y = charucoCorners.at(1).y - midP.y;

                                    // Tager midt funde midt punkt på linjen og addere den funde roteret vector så den står vinkelret på linjen.
                                    p6.y = rotatedVec.x * (-1) + midP.y;
                                    p6.x = rotatedVec.y + midP.x;
                                    cv::line(inputImage, midP, p6, cv::Scalar(0,0,255), 4);

                                    p7.x = (rotatedVec.y * (-1)) + midP.x;
                                    p7.y = (rotatedVec.x * (-1)) * (-1) + midP.y;
                                    cv::line(inputImage, midP, p7, cv::Scalar(0,0,255), 4);

                                    sharpVec.x = (p7.x - p6.x) + p6.x;
                                    sharpVec.y = (p7.y - p6.y) + p6.y;
                                    cout << "sharpVec x: " << sharpVec.x << " sharpVec y: " << sharpVec.y << endl;
                                    cv::line(inputImage, p6, p7, cv::Scalar(0,0,255), 4);




                                    vector<Point2f> sharpHist;

                                    cv::LineIterator it(temp, p6, p7, 8,false);
                                    vector<int> buf;
                                    //buf.resize(it.count);
                                    for (int i=0; i<it.count;i++, ++it)
                                    {
                                        //cout << "Linje koordinator: " << it.pos() << endl;
                                        buf.push_back((int)temp.at<uchar>(it.pos().x,it.pos().y));
                                        //cout << buf.at(i) << endl;

                                    }

                                    float range[] = { 0, 256 }; //the upper boundary is exclusive
                                    const float* histRange[] = { range };
                                    int histSize = it.count;
                                    int hist_w = 512;
                                    int hist_h = 400;
                                    int bin_w = cvRound( (double) hist_w/histSize );

                                    Mat histImage(512, 400, CV_8UC1, Scalar( 0,0,0) );

                                    for( int i = 1; i < histSize; i++ )
                                    {
                                        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(buf.at(i-1)) ), Point( bin_w*(i), hist_h - cvRound(buf.at(i)) ), Scalar( 255, 0, 0), 2, 8, 0  );
                                    }
                                    imshow("calcHist Demo", histImage );
                                }
                            }
                        }
                    }
                    namedWindow("charucoMarkers", 1);
                    cv::imshow("charucoMarkers", inputImage);
                    waitKey(0);
                }
            }
            catch (Exception& e)
            {
                qDebug() << e.what() << Qt::endl;
            }
        }
        else
        {
            cout << "Ingen billeder og/eller paths" << endl;
        }
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


