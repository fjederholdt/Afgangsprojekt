#include "analyse.h"
#include "ui_analyse.h"
#include "dhparams.h"
#include "fsclass.h"
#include "kalibreringsFunktioner.h"
#include "kameraFunktioner.h"

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
        string robotPosesCSV = kalibPath+"/robotPoses.csv";

        Mat grayImage, cameraMatrix, distCoeffs, camRotationMatrix, camTranslationMatrix, hErotationMatrix, hEtranslationMatrix;

        FSClass fsCamera(kalibPath+"/cameraData.yml", datoStr.toStdString());
        FSClass fsHandEye(kalibPath+"/handEyeData.yml", datoStr.toStdString());

        fsCamera.readCamera(cameraMatrix, distCoeffs, camRotationMatrix, camTranslationMatrix);
        fsHandEye.readHandEye(hErotationMatrix, hEtranslationMatrix);


        fstream fin;
        fin.open(robotPosesCSV, ios::in);
        vector<string> rowPose;
        string line, word;
        vector<vector<double>> robotPoses;
        while (robotPoses.size() == 0)
        {
            vector<double> pose;
            getline(fin, line);
            stringstream s(line);
            while(getline(s, word, ','))
            {
                size_t foundPng = word.find(".png");
                size_t foundRobot = word.find("robot");
                if(foundPng == std::string::npos && foundRobot == std::string::npos)
                {
                    rowPose.push_back(word);
                }
            }
            if(!rowPose.empty())
            {
                for	(size_t i = 0; i < rowPose.size(); i++)
                {
                    pose.push_back(stod(rowPose.at(i)));
                }
                robotPoses.push_back(pose);
            }
        }

        RTDEReceiveInterface rtde_receive(hostname);
        RTDEControlInterface rtde_control(hostname);

        rtde_control.moveJ(robotPoses.at(0));

        std::vector<double> joint_positions = rtde_receive.getActualQ();
        for (size_t i = 0; i < joint_positions.size(); i++)
        {
            cout << joint_positions.at(i) << endl;

        }
        std::vector<double> cartesian_positions = rtde_receive.getActualTCPPose();
        for (size_t i = 0; i < cartesian_positions.size(); i++)
        {
            cout << cartesian_positions.at(i) << endl;
        }

        tagBillede(grayImage, false);

        vector<Mat> grayVector, grayRemap;
        grayVector.push_back(grayImage);

        //remapping(grayVector, cameraMatrix, distCoeffs, map1, map2, grayRemap);

        vector<Mat> rMats;
        vector<Mat> tMats;
        vector<Point2f> charucoCorners;
        vector<int> charucoIds;

        Mat rvec, rmat, tmat;
        Mat tvec(4,1,6);

        vector<vector<Point3f>> objectPoints = board->objPoints;
        vector<vector<Point2f>> markerCorners;
        vector<int> markerIds;

        aruco::detectMarkers(grayImage, board->dictionary, markerCorners, markerIds, params);

        aruco::interpolateCornersCharuco(markerCorners, markerIds, grayImage, board, charucoCorners, charucoIds);

        aruco::drawDetectedCornersCharuco(grayImage, charucoCorners, charucoIds);

        solvePnP(Mat(objectPoints.at(0)), Mat(markerCorners.at(0)), cameraMatrix, distCoeffs, rvec, tmat);

        cout << "rvec: " << rvec << endl;
        Rodrigues(rvec, rmat);
        cout << "rmat: " << rmat << endl;
        cout << "tmat: " << tmat << endl;


        namedWindow("gray", 1);
        imshow("gray", grayImage);

        tvec.at<double>(0) = 0;//charucoCorners.at(0).x*0.00023;
        tvec.at<double>(1) = 0;//charucoCorners.at(0).y*0.00023;
        tvec.at<double>(2) = 0;
        tvec.at<double>(3) = 1;

        cout << "tvec: " << tvec << endl;

        charucoPoints(grayVector, cameraMatrix, distCoeffs, rMats, tMats);

        cout << "rmat: " << rMats.at(0) << endl;
        cout << "tmat: " << tMats.at(0) << endl;

        vector<Mat> multi;
        Mat robot2Target(4,4,CV_64F);
        Mat handEyeMat (4,4,CV_64F);
        Mat cameraMat(4,4,CV_64F);
        Mat cameraMat2(4,4,CV_64F);
        Mat solvepnpMat(4,4,CV_64F);
        Mat base2TCP(4,4,CV_64F);
        DHParams dhp(joint_positions);
        dhp.calculateDH();

        cout << "robot: " << dhp.getMatrix() << endl;
        base2TCP = dhp.getMatrix();

        multi.push_back(base2TCP);

        handEyeMat.at<double>(0) = hErotationMatrix.at<double>(0);
        handEyeMat.at<double>(1) = hErotationMatrix.at<double>(1);
        handEyeMat.at<double>(2) = hErotationMatrix.at<double>(2);
        handEyeMat.at<double>(3) = hEtranslationMatrix.at<double>(0);
        handEyeMat.at<double>(4) = hErotationMatrix.at<double>(3);
        handEyeMat.at<double>(5) = hErotationMatrix.at<double>(4);
        handEyeMat.at<double>(6) = hErotationMatrix.at<double>(5);
        handEyeMat.at<double>(7) = hEtranslationMatrix.at<double>(1);
        handEyeMat.at<double>(8) = hErotationMatrix.at<double>(6);
        handEyeMat.at<double>(9) = hErotationMatrix.at<double>(7);
        handEyeMat.at<double>(10) = hErotationMatrix.at<double>(8);
        handEyeMat.at<double>(11) = hEtranslationMatrix.at<double>(2);
        handEyeMat.at<double>(12) = 0;
        handEyeMat.at<double>(13) = 0;
        handEyeMat.at<double>(14) = 0;
        handEyeMat.at<double>(15) = 1;

        cout << "handeye: " << handEyeMat << endl;
        multi.push_back(handEyeMat);

        cameraMat.at<double>(0) = camRotationMatrix.at<double>(0);
        cameraMat.at<double>(1) = camRotationMatrix.at<double>(1);
        cameraMat.at<double>(2) = camRotationMatrix.at<double>(2);
        cameraMat.at<double>(3) = camTranslationMatrix.at<double>(0);
        cameraMat.at<double>(4) = camRotationMatrix.at<double>(3);
        cameraMat.at<double>(5) = camRotationMatrix.at<double>(4);
        cameraMat.at<double>(6) = camRotationMatrix.at<double>(5);
        cameraMat.at<double>(7) = camTranslationMatrix.at<double>(1);
        cameraMat.at<double>(8) = camRotationMatrix.at<double>(6);
        cameraMat.at<double>(9) = camRotationMatrix.at<double>(7);
        cameraMat.at<double>(10) = camRotationMatrix.at<double>(8);
        cameraMat.at<double>(11) = camTranslationMatrix.at<double>(2);
        cameraMat.at<double>(12) = 0;
        cameraMat.at<double>(13) = 0;
        cameraMat.at<double>(14) = 0;
        cameraMat.at<double>(15) = 1;

        cout << "cameraMat: " << cameraMat << endl;

        cameraMat2.at<double>(0) = rMats.at(0).at<double>(0);
        cameraMat2.at<double>(1) = rMats.at(0).at<double>(1);
        cameraMat2.at<double>(2) = rMats.at(0).at<double>(2);
        cameraMat2.at<double>(3) = tMats.at(0).at<double>(0);
        cameraMat2.at<double>(4) = rMats.at(0).at<double>(3);
        cameraMat2.at<double>(5) = rMats.at(0).at<double>(4);
        cameraMat2.at<double>(6) = rMats.at(0).at<double>(5);
        cameraMat2.at<double>(7) = tMats.at(0).at<double>(1);
        cameraMat2.at<double>(8) = rMats.at(0).at<double>(6);
        cameraMat2.at<double>(9) = rMats.at(0).at<double>(7);
        cameraMat2.at<double>(10) = rMats.at(0).at<double>(8);
        cameraMat2.at<double>(11) = tMats.at(0).at<double>(2);
        cameraMat2.at<double>(12) = 0;
        cameraMat2.at<double>(13) = 0;
        cameraMat2.at<double>(14) = 0;
        cameraMat2.at<double>(15) = 1;

        cout << "cameraMat2: " << cameraMat2 << endl;

        solvepnpMat.at<double>(0) = rmat.at<double>(0);
        solvepnpMat.at<double>(1) = rmat.at<double>(1);
        solvepnpMat.at<double>(2) = rmat.at<double>(2);
        solvepnpMat.at<double>(3) = tmat.at<double>(0);
        solvepnpMat.at<double>(4) = rmat.at<double>(3);
        solvepnpMat.at<double>(5) = rmat.at<double>(4);
        solvepnpMat.at<double>(6) = rmat.at<double>(5);
        solvepnpMat.at<double>(7) = tmat.at<double>(1);
        solvepnpMat.at<double>(8) = rmat.at<double>(6);
        solvepnpMat.at<double>(9) = rmat.at<double>(7);
        solvepnpMat.at<double>(10) = rmat.at<double>(8);
        solvepnpMat.at<double>(11) = tmat.at<double>(2);
        solvepnpMat.at<double>(12) = 0;
        solvepnpMat.at<double>(13) = 0;
        solvepnpMat.at<double>(14) = 0;
        solvepnpMat.at<double>(15) = 1;

        cout << "solvepnp: " << solvepnpMat << endl;

        Mat zRot = Mat::zeros(4,4,6);

        zRot.at<double>(0,0) = 1;
        zRot.at<double>(1,1) = -1;
        zRot.at<double>(2,2) = 1;
        zRot.at<double>(3,3) = 1;

        Mat newRot = cameraMat * zRot;

        cout << "newRot: " << newRot << endl;

        multi.push_back(cameraMat2);


        robot2Target = dhp.multiplyDH(multi);

        cout << "robot2Target: " << robot2Target << endl;

        Mat robot2Target2 = base2TCP * handEyeMat * cameraMat2;

         cout << "robot2Target without dhp: " << robot2Target2 << endl;

        Mat displacement;

        cout << "displacement: " << endl;
        displacement = robot2Target* tvec;

        cout << displacement << endl;

       /* cout << "displacement2: " << endl;
        displacement = newRot* tvec;

        cout << displacement << endl;*/

        vector<double> IKpoints;

       /* IKpoints.push_back(displacement.at<double>(0));
        IKpoints.push_back(displacement.at<double>(1));
        IKpoints.push_back(displacement.at<double>(2));
        IKpoints.push_back(robRvec.at<double>(0));
        IKpoints.push_back(robRvec.at<double>(1));
        IKpoints.push_back(robRvec.at<double>(2));

        //cartesian_positions.at(2) -=30;
        for(size_t i = 0; i < IKpoints.size(); i++)
        {
            cout <<  IKpoints.at(i) << endl;
        }af
        */

        cartesian_positions.at(0) = displacement.at<double>(0);
        cartesian_positions.at(1) = displacement.at<double>(1);
        cartesian_positions.at(2) = displacement.at<double>(2)+0.30;
        //rtde_control.moveL(cartesian_positions);

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

Mat getRm(Mat matrix)
{
    Mat rvec(3,1,6);
    Mat rmat(3,3,6);

    rmat.at<double>(0) = matrix.at<double>(0);
    rmat.at<double>(1) = matrix.at<double>(1);
    rmat.at<double>(2) = matrix.at<double>(2);
    rmat.at<double>(3) = matrix.at<double>(4);
    rmat.at<double>(4) = matrix.at<double>(5);
    rmat.at<double>(5) = matrix.at<double>(6);
    rmat.at<double>(6) = matrix.at<double>(8);
    rmat.at<double>(7) = matrix.at<double>(9);
    rmat.at<double>(8) = matrix.at<double>(10);

    Rodrigues(rmat, rvec);
    return rvec;
}

Mat getTm(Mat matrix)
{
    Mat tvec(3,1,6);

    tvec.at<double>(0) = matrix.at<double>(3);
    tvec.at<double>(1) = matrix.at<double>(7);
    tvec.at<double>(2) = matrix.at<double>(11);

    return tvec;
}

pcl::visualization::PCLVisualizer::Ptr viz (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, vector<Mat> cVRMatrix, vector<Mat> cVTMatrix){
    Eigen::Affine3f affine = Eigen::Affine3f::Identity();

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

    /*for(size_t i = 0; i < cloud->size(); i++)
    {
        Mat rvec;
        Mat rmat = cVRMatrix.at(i);
        Rodrigues(rmat, rvec);
        Mat tvec = cVTMatrix.at(i);
        float thetaX = rvec.at<float>(0);
        float thetaY = rvec.at<float>(1);
        float thetaZ = rvec.at<float>(2);

        /*affine.rotate (Eigen::AngleAxisf (thetaX, Eigen::Vector3f::UnitX()));
        affine.rotate (Eigen::AngleAxisf (thetaY, Eigen::Vector3f::UnitY()));
        affine.rotate (Eigen::AngleAxisf (thetaZ, Eigen::Vector3f::UnitZ()));

        affine.translation()[0] = tvec.at<float>(0);
        affine.translation()[1] = tvec.at<float>(1);
        affine.translation()[2] = tvec.at<float>(2);

        viewer->addCoordinateSystem(1, affine);
    }*/

    viewer->addCoordinateSystem(1);
    viewer->initCameraParameters();
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
        string robotPosesCSV = kalibPath+"/robotPoses.csv";
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

        fstream fin;
        fin.open(robotPosesCSV, ios::in);
        vector<string> rowPose;
        string word;
        vector<vector<double>> robotPoses;
        while (robotPoses.size() == 0)
        {
            vector<double> pose;
            getline(fin, line);
            stringstream s(line);
            while(getline(s, word, ','))
            {
                size_t foundPng = word.find(".png");
                size_t foundRobot = word.find("robot");
                if(foundPng == std::string::npos && foundRobot == std::string::npos)
                {
                    rowPose.push_back(word);
                }
            }
            if(!rowPose.empty())
            {
                for	(size_t i = 0; i < rowPose.size(); i++)
                {
                    pose.push_back(stod(rowPose.at(i)));
                }
                robotPoses.push_back(pose);
            }
        }

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
        FSClass fsHandEye(kalibPath+"/handEyeData.yml", datoStr.toStdString());

        vector<Mat> cVRMatrix, cVTMatrix;
        Mat cameraMatrix, distCoeffs, camRotationMatrix, camTranslationMatrix;
        Mat robotRm, robotTm;
        Mat handEyeRm, handEyeTm;

        fsCamera.readCamera(cameraMatrix, distCoeffs, camRotationMatrix, camTranslationMatrix);
        cVRMatrix.push_back(camRotationMatrix);
        cVTMatrix.push_back(camTranslationMatrix);
        fsHandEye.readHandEye(handEyeRm, handEyeTm);


        DHParams dhp(robotPoses.at(0));
        dhp.calculateDH();

        robotTm = dhp.getTM();
        robotRm = dhp.getRM();

        cVTMatrix.push_back(robotTm);
        cVTMatrix.push_back(handEyeTm);

        cVRMatrix.push_back(robotRm);
        cVRMatrix.push_back(handEyeRm);


        pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::PointXYZ robot;
        pcl::PointXYZ camera;
        pcl::PointXYZ handeye;

        camera._PointXYZ::x = camTranslationMatrix.at<double>(0);
        camera._PointXYZ::y = camTranslationMatrix.at<double>(1);
        camera._PointXYZ::z = camTranslationMatrix.at<double>(2);

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

        viewer = viz((pcl::PointCloud<pcl::PointXYZ>::ConstPtr)basic_cloud_ptr, cVRMatrix, cVTMatrix);

        while(!viewer->wasStopped())
        {
            viewer->spinOnce(100);
        }
        if(viewer->wasStopped())
        {
            viewer->close();
        }
        //qDebug() << kalibPath;


/*
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
        Mat cameraMatrix, distCoeffs, camRotationMatrix, camTranslationMatrix;
        fsCamera.readCamera(cameraMatrix, distCoeffs, camRotationMatrix, camTranslationMatrix);

        grayImages = getImages(billedePath);

        Mat map1, map2;
        vector<Mat> rview;

        remapping(grayImages, cameraMatrix, distCoeffs, map1, map2, rview);

        if (!grayImages.empty())
        {
            try
            {
                std::vector<int> markerIds;
                std::vector<std::vector<cv::Point2f> > markerCorners;

                std::vector<cv::Point2f> charucoCorners;
                std::vector<int> charucoIds;


                for(vector<Mat>::iterator iter = rview.begin(); iter != rview.end(); iter++)
                {
                    Mat inputImage = *iter;
                    cv::aruco::detectMarkers(inputImage, board->dictionary, markerCorners, markerIds, params);
                    if (markerIds.size() > 0)
                    {
                        cv::aruco::drawDetectedMarkers(inputImage, markerCorners, markerIds, cv::Scalar(255,0,0));
                        std::vector<cv::Point2f> charucoCorners;
                        std::vector<int> charucoIds;
                        cv::aruco::interpolateCornersCharuco(markerCorners, markerIds, grayImages.at(0), board, charucoCorners, charucoIds);
                                // if at least one charuco corner detected
                        if (charucoIds.size() > 0)
                        {
                            cv::aruco::drawDetectedCornersCharuco(inputImage, charucoCorners, charucoIds, cv::Scalar(255, 0, 0));
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


