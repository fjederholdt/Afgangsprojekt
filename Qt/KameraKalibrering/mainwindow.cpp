#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "kalibrering.h"
#include "analyse.h"

#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->camera_frame->setStyleSheet("background-color: rgb(255, 0, 0);");
    //connect(ui->cameraButton, &QPushButton::clicked, this, &MainWindow::onOpenGL);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setValg(QString valgte)
{
    valg=valgte;
}

void MainWindow::on_Kalibrering_clicked()
{
    Kalibrering kalibrering;
    kalibrering.setModal(true);
    kalibrering.setWindowTitle("Kalibrering");
    kalibrering.exec();
}

void MainWindow::on_Analyse_clicked()
{
    QMessageBox msg;
    msg.setText("Ingen kalibrering valgt, tryk på {Kalibrering} for at vælge kalibrering");
    Analyse analyse;
    analyse.setValg(valg);
    msg.setText(valg);
    analyse.setModal(true);
    analyse.setWindowTitle("Analyse");
    if(analyse.getValg().isEmpty())
    {
        msg.exec();
    }
    else
    {
        analyse.exec();
    }
}


void MainWindow::on_cameraButton_clicked()
{
    if(!camera)
    {
        camera = true;
        ui->camera_frame->setStyleSheet("background-color: rgb(0, 128, 0);");
        VideoCapture capture(0, CAP_ANY);

            if (!capture.isOpened())
            {
                cout << "could not open camera" << endl;
                return;
            }

            Size frame = Size((int)capture.get(CAP_PROP_FRAME_WIDTH), (int)capture.get(CAP_PROP_FRAME_HEIGHT));

            namedWindow("Camera feed", WINDOW_AUTOSIZE);
            moveWindow("Camera feed", 400, 0);

            Mat window;
            for(;;)
            {
                capture >> window;

                if (window.empty())
                {
                    cout << "empty frame" << endl;
                    break;
                }

                imshow("Camera feed", window);

                char c = (char)waitKey(2 );
                if (c == 27) break;
            }
            destroyAllWindows();
            camera = false;
            ui->camera_frame->setStyleSheet("background-color: rgb(255, 0, 0);");
    }
    else
    {
        camera = false;
        ui->camera_frame->setStyleSheet("background-color: rgb(255, 0, 0);");
    }
/*
    if(camera)
    {


                uint video;
                glGenTextures(1,&video);
                glBindTexture(GL_TEXTURE_2D, video);
                glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT );
                glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT );
                glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
                glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );


               // namedWindow("window");
                //imshow("window",window);
                //Mat signedMat;
                //window.convertTo(signedMat, CV_8SC1);
                //QByteArray arr = QByteArray::fromRawData((char*)signedMat.data,sizeof(signedMat.size));

                //window.convertTo(win,CV_8U);


                //QPainter painter);
                //painter.drawPixmap(10, 10, pix);



                waitKey(1);



            //}
        }
    }*/

}


