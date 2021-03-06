#ifndef NYKALIBRERING_H
#define NYKALIBRERING_H

#include <QDialog>
#include <QDialogButtonBox>
#include <QDebug>

#include <bits/stdc++.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <filesystem>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include<unistd.h>

#include <boost/filesystem.hpp>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/aruco/dictionary.hpp>
#include <opencv2/opencv.hpp>

#include <pylon/PylonBase.h>
#include <pylon/PylonIncludes.h>
#include <pylon/TlFactory.h>
#include <pylon/ImageFormatConverter.h>
#include <pylon/PylonImage.h>

#include <ur_rtde/rtde.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>

#include "fsclass.h"


namespace Ui {
class NyKalibrering;
}

class NyKalibrering : public QDialog
{
    Q_OBJECT

public:
    explicit NyKalibrering(QWidget *parent = nullptr);
    ~NyKalibrering();
    void addList(std::string strItem);
    void removeList();
    void setPath(std::string &kalibpath);
    void tagBillede();

private slots:
    void on_tag_billede_clicked();

    void on_slet_billede_clicked();

    void on_valg_alle_billeder_clicked();

    void on_brug_vs_clicked();

    void on_kalibrere_clicked();

    void on_annuller_clicked();

    void on_auto_billede_clicked();

    void on_Charucomarkers_clicked();

private:
    Ui::NyKalibrering *ui;
    std::string path;
    std::string hostname = "192.168.250.1";
    int image_nr = 0;
    std::string localTime;
    std::ofstream output_file;
};

#endif // NYKALIBRERING_H
