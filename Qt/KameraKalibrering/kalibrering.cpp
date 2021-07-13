#include "kalibrering.h"
#include "ui_kalibrering.h"
#include "kalibreringsFunktioner.h"
#include "nykalibrering.h"
#include "mainwindow.h"

using namespace std::filesystem;
using namespace cv;
using namespace std;

Kalibrering::Kalibrering(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Kalibrering)
{
    ui->setupUi(this);
}

Kalibrering::~Kalibrering()
{
    delete ui;
}

void Kalibrering::setPath(const QString &mainPath)
{
    this->folderpath = mainPath.toStdString();
    std::vector<std::string> pathVector;
    std::string sub, suffix;
    for(const auto & entry : directory_iterator(folderpath))
    {
        sub = entry.path();
        std::size_t found = sub.find_last_of("/");
        sub = sub.substr(found+1,sub.size());
        pathVector.push_back(sub);
    }
    for (size_t i = 0; i < pathVector.size(); i++)
    {
        QListWidgetItem *item = new QListWidgetItem;
        item->setText(QString::fromStdString(pathVector.at(i)));
        ui->listWidget->addItem(item);
    }
}

void Kalibrering::on_ny_kalibrering_clicked()
{
    time_t rawtime;
    time(&rawtime);
    string localTime = asctime(localtime(&rawtime));
    localTime.pop_back();
    NyKalibrering nyKalibrering;
    nyKalibrering.setModal(true);
    nyKalibrering.setWindowTitle(QString::fromStdString("Kalibrering: "+ localTime));
    nyKalibrering.setPath(folderpath);
    nyKalibrering.exec();

    //efter nyKalibrering er lukket
    vector<std::string> pathVector;
    string sub, suffix;
    for(const auto & entry : directory_iterator(folderpath))
    {
        sub = entry.path();
        size_t found = sub.find_last_of("/");
        sub = sub.substr(found+1,sub.size());
        pathVector.push_back(sub);
    }
    ui->listWidget->clear();
    for (size_t i = 0; i < pathVector.size(); i++)
    {
        QListWidgetItem *item = new QListWidgetItem;
        item->setText(QString::fromStdString(pathVector.at(i)));
        ui->listWidget->addItem(item);
    }
}

void Kalibrering::on_annuller_clicked()
{
    Kalibrering::close();
}

void Kalibrering::removeKalib()
{
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    foreach(QListWidgetItem* item, items)
        delete ui->listWidget->takeItem(ui->listWidget->row(item));
}

void Kalibrering::on_slet_kalibrering_clicked()
{
    vector<string> pathVector;
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    for(int i = 0; i < items.size(); i++)
        pathVector.push_back(folderpath+"/"+items.at(i)->text().toStdString());
    for(size_t i = 0; i < pathVector.size(); i++)
    {
        boost::filesystem::remove_all(pathVector.at(i));
    }
    removeKalib();
}

void Kalibrering::on_kalibrere_clicked()
{
    vector<std::string> billeder;
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    if(!items.empty())
    {
        try
        {
            string time = items.at(0)->text().toStdString();
            string billedePath = folderpath+time;
            string robotPosesCSV = billedePath+"/robotPoses.csv";
            set<path> sorted;
            for(const auto & entry : directory_iterator(billedePath))
            {
                string sub = entry.path();
                size_t found = sub.find_last_of(".");
                sub = sub.substr(found,sub.size());
                if(sub == ".png")
                {
                    sorted.insert(entry.path());
                }
            }
            for (auto &filename : sorted )
            {
                billeder.push_back(filename.c_str());
            }

            vector<Mat> images = getImages(billeder);
            fstream fin;
            fin.open(robotPosesCSV, ios::in);
            vector<string> row;
            string line, word;
            vector<vector<double>> robotPoses;
            while (robotPoses.size() < images.size())
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
                            png = billedePath+"/"+word;
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
                            //cout << word
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

            FSClass fsCamera(billedePath+"/cameraData.yml", time);
            FSClass fsRobot(billedePath+"/robotData.yml", time);
            FSClass fsHandEye(billedePath+"/handEyeData.yml", time);

            Mat cameraMatrix, distCoeffs, map1, map2;
            vector<Mat> rvectors, tvectors, imagesRemap, rotationMat, translationMat;
            vector<vector<int>> charucoIds;
            vector<vector<Point2f>> charucoCorners;

            vector<double> repErrors = calibrateCharuco(images, cameraMatrix, distCoeffs, charucoCorners, charucoIds, rotationMat, translationMat);

            Mat rodRotationMat;
            Rodrigues(rotationMat.at(0),rodRotationMat);
            fsCamera.writeCamera(cameraMatrix, distCoeffs, rodRotationMat, translationMat.at(0));

            ofstream repErr;
            repErr.open(billedePath+"/repError.txt");
            repErr << "repErrors: " << endl;
            for(size_t i = 0; i < repErrors.size(); i++)
            {
                repErr << "image number: " << i << " = " << repErrors.at(i) << endl;
            }

            repErr.close();

            remapping(images, cameraMatrix, distCoeffs, map1, map2, imagesRemap);

            //charucoBoardPose(images, cameraMatrix, distCoeffs, charucoCorners, charucoIds, rvectors, tvectors);

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
            msg.exec();
        }
        catch (Exception& e)
        {
            qDebug() << "Exception thrown: " << e.what();
        }
    }
    else
    {
        QMessageBox msg;
        msg.setText("Ingen Kalibrering er valgt");
        msg.exec();
    }
}

void Kalibrering::on_pushButton_clicked() // Mean og standard deviation
{
    cout << board->objPoints.at(0).at(0).x << ", " << board->objPoints.at(0).at(0).y;

   /* vector<string> pathVector;
    vector<string> billeder;
    QList<QListWidgetItem*> items = ui->listWidget->selectedItems();
    for(int i = 0; i < items.size(); i++)
        pathVector.push_back(folderpath+"/"+items.at(i)->text().toStdString());

    string sub;
    for(const auto & entry : directory_iterator(pathVector.at(0)))
    {
        sub = entry.path();
        std::size_t found = sub.find_last_of(".");
        sub = sub.substr(found,sub.size());
        if(sub == ".png")
        {
            billeder.push_back(entry.path());
        }
    }

    vector<Mat> images = getImages(billeder);
    Mat mean, stdDev;
    for (size_t i = 0; i < images.size(); i++)
    {
        meanStdDev(images.at(i), mean, stdDev);
        cout << "billede: " << i << " mean: " << mean << endl;
        cout << "standard deviation: " << stdDev << endl;
    }*/
}

