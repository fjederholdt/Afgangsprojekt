#ifndef ANALYSE_H
#define ANALYSE_H

#include <QDialog>
#include <QDebug>
#include <QMessageBox>
#include <QApplication>
#include <QHeaderView>
#include <QFormLayout>
#include <QPixmap>
#include <iostream>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>
#include <time.h>
#include <stdio.h>
#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>
//#include <pcl/features/normal_3d.h>

#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>


#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>
#include <string>


namespace Ui {
class Analyse;
}

class Analyse : public QDialog
{
    Q_OBJECT

public:
    explicit Analyse(QWidget *parent = nullptr);
    ~Analyse();
    void setPath(std::string &mainPath);

private slots:
    void on_anvend_valgte_kalibrering_clicked();

    void on_test_kalibrering_clicked();

    void on_visualiser_clicked();

    void on_annuller_clicked();

    void on_hvis_charuco_clicked();

private:
    Ui::Analyse *ui;
    std::string path;
    std::string hostname = "192.168.250.1";

};

#endif // ANALYSE_H
