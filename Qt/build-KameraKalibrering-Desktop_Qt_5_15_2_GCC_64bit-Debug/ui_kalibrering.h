/********************************************************************************
** Form generated from reading UI file 'kalibrering.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_KALIBRERING_H
#define UI_KALIBRERING_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_Kalibrering
{
public:
    QPushButton *ny_kalibrering;
    QListWidget *listWidget;
    QPushButton *annuller;
    QPushButton *slet_kalibrering;

    void setupUi(QDialog *Kalibrering)
    {
        if (Kalibrering->objectName().isEmpty())
            Kalibrering->setObjectName(QString::fromUtf8("Kalibrering"));
        Kalibrering->resize(640, 460);
        ny_kalibrering = new QPushButton(Kalibrering);
        ny_kalibrering->setObjectName(QString::fromUtf8("ny_kalibrering"));
        ny_kalibrering->setGeometry(QRect(328, 60, 151, 71));
        listWidget = new QListWidget(Kalibrering);
        listWidget->setObjectName(QString::fromUtf8("listWidget"));
        listWidget->setGeometry(QRect(30, 50, 256, 192));
        annuller = new QPushButton(Kalibrering);
        annuller->setObjectName(QString::fromUtf8("annuller"));
        annuller->setGeometry(QRect(340, 260, 121, 61));
        slet_kalibrering = new QPushButton(Kalibrering);
        slet_kalibrering->setObjectName(QString::fromUtf8("slet_kalibrering"));
        slet_kalibrering->setGeometry(QRect(340, 170, 131, 51));

        retranslateUi(Kalibrering);

        QMetaObject::connectSlotsByName(Kalibrering);
    } // setupUi

    void retranslateUi(QDialog *Kalibrering)
    {
        Kalibrering->setWindowTitle(QCoreApplication::translate("Kalibrering", "Dialog", nullptr));
        ny_kalibrering->setText(QCoreApplication::translate("Kalibrering", "Ny kalibrering", nullptr));
        annuller->setText(QCoreApplication::translate("Kalibrering", "F\303\246rdig", nullptr));
        slet_kalibrering->setText(QCoreApplication::translate("Kalibrering", "Slet kalibrering", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Kalibrering: public Ui_Kalibrering {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_KALIBRERING_H
