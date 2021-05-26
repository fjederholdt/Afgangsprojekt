/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QFrame *robot_frame;
    QFrame *camera_frame;
    QLabel *camera_connected;
    QLabel *robot_connected;
    QPushButton *cameraButton;
    QPushButton *robotButton;
    QPushButton *Analyse;
    QPushButton *Kalibrering;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->setWindowModality(Qt::ApplicationModal);
        MainWindow->resize(800, 600);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        robot_frame = new QFrame(centralwidget);
        robot_frame->setObjectName(QString::fromUtf8("robot_frame"));
        robot_frame->setGeometry(QRect(50, 100, 21, 21));
        robot_frame->setFrameShape(QFrame::StyledPanel);
        robot_frame->setFrameShadow(QFrame::Raised);
        camera_frame = new QFrame(centralwidget);
        camera_frame->setObjectName(QString::fromUtf8("camera_frame"));
        camera_frame->setGeometry(QRect(50, 60, 21, 21));
        camera_frame->setFrameShape(QFrame::StyledPanel);
        camera_frame->setFrameShadow(QFrame::Raised);
        camera_connected = new QLabel(centralwidget);
        camera_connected->setObjectName(QString::fromUtf8("camera_connected"));
        camera_connected->setGeometry(QRect(110, 60, 131, 17));
        robot_connected = new QLabel(centralwidget);
        robot_connected->setObjectName(QString::fromUtf8("robot_connected"));
        robot_connected->setGeometry(QRect(110, 100, 131, 17));
        cameraButton = new QPushButton(centralwidget);
        cameraButton->setObjectName(QString::fromUtf8("cameraButton"));
        cameraButton->setGeometry(QRect(270, 60, 89, 25));
        robotButton = new QPushButton(centralwidget);
        robotButton->setObjectName(QString::fromUtf8("robotButton"));
        robotButton->setGeometry(QRect(270, 100, 89, 25));
        Analyse = new QPushButton(centralwidget);
        Analyse->setObjectName(QString::fromUtf8("Analyse"));
        Analyse->setGeometry(QRect(490, 360, 161, 81));
        Kalibrering = new QPushButton(centralwidget);
        Kalibrering->setObjectName(QString::fromUtf8("Kalibrering"));
        Kalibrering->setGeometry(QRect(110, 360, 161, 81));
        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 800, 22));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "MainWindow", nullptr));
        camera_connected->setText(QCoreApplication::translate("MainWindow", "Camera connected", nullptr));
        robot_connected->setText(QCoreApplication::translate("MainWindow", "Robot connected", nullptr));
        cameraButton->setText(QCoreApplication::translate("MainWindow", "On/Off", nullptr));
        robotButton->setText(QCoreApplication::translate("MainWindow", "On/Off", nullptr));
        Analyse->setText(QCoreApplication::translate("MainWindow", "Analyse", nullptr));
        Kalibrering->setText(QCoreApplication::translate("MainWindow", "Kalibrering", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
