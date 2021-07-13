/********************************************************************************
** Form generated from reading UI file 'analyse.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ANALYSE_H
#define UI_ANALYSE_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTableWidget>

QT_BEGIN_NAMESPACE

class Ui_Analyse
{
public:
    QTableWidget *tableWidget;
    QPushButton *visualiser;
    QPushButton *anvend_valgte_kalibrering;
    QPushButton *test_kalibrering;
    QPushButton *annuller;
    QLabel *imageLabel;
    QLabel *label;
    QLabel *maxRep;
    QPushButton *hvis_charuco;

    void setupUi(QDialog *Analyse)
    {
        if (Analyse->objectName().isEmpty())
            Analyse->setObjectName(QString::fromUtf8("Analyse"));
        Analyse->resize(1066, 489);
        tableWidget = new QTableWidget(Analyse);
        tableWidget->setObjectName(QString::fromUtf8("tableWidget"));
        tableWidget->setGeometry(QRect(20, 20, 391, 181));
        tableWidget->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        tableWidget->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        tableWidget->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContents);
        visualiser = new QPushButton(Analyse);
        visualiser->setObjectName(QString::fromUtf8("visualiser"));
        visualiser->setGeometry(QRect(450, 70, 89, 25));
        anvend_valgte_kalibrering = new QPushButton(Analyse);
        anvend_valgte_kalibrering->setObjectName(QString::fromUtf8("anvend_valgte_kalibrering"));
        anvend_valgte_kalibrering->setGeometry(QRect(60, 290, 201, 25));
        test_kalibrering = new QPushButton(Analyse);
        test_kalibrering->setObjectName(QString::fromUtf8("test_kalibrering"));
        test_kalibrering->setGeometry(QRect(370, 290, 141, 25));
        annuller = new QPushButton(Analyse);
        annuller->setObjectName(QString::fromUtf8("annuller"));
        annuller->setGeometry(QRect(240, 380, 89, 25));
        imageLabel = new QLabel(Analyse);
        imageLabel->setObjectName(QString::fromUtf8("imageLabel"));
        imageLabel->setGeometry(QRect(666, 60, 291, 291));
        imageLabel->setFrameShape(QFrame::Box);
        imageLabel->setFrameShadow(QFrame::Plain);
        imageLabel->setLineWidth(3);
        imageLabel->setMidLineWidth(0);
        label = new QLabel(Analyse);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(710, 20, 201, 17));
        maxRep = new QLabel(Analyse);
        maxRep->setObjectName(QString::fromUtf8("maxRep"));
        maxRep->setGeometry(QRect(690, 380, 271, 17));
        hvis_charuco = new QPushButton(Analyse);
        hvis_charuco->setObjectName(QString::fromUtf8("hvis_charuco"));
        hvis_charuco->setGeometry(QRect(370, 330, 141, 21));

        retranslateUi(Analyse);

        QMetaObject::connectSlotsByName(Analyse);
    } // setupUi

    void retranslateUi(QDialog *Analyse)
    {
        Analyse->setWindowTitle(QCoreApplication::translate("Analyse", "Dialog", nullptr));
        visualiser->setText(QCoreApplication::translate("Analyse", "Visualiser", nullptr));
        anvend_valgte_kalibrering->setText(QCoreApplication::translate("Analyse", "Anvned valgte kalibrering", nullptr));
        test_kalibrering->setText(QCoreApplication::translate("Analyse", "Test kalibrering", nullptr));
        annuller->setText(QCoreApplication::translate("Analyse", "F\303\246rdig", nullptr));
        imageLabel->setText(QString());
        label->setText(QCoreApplication::translate("Analyse", "graf over reprojection errors", nullptr));
        maxRep->setText(QCoreApplication::translate("Analyse", "Max reperror:", nullptr));
        hvis_charuco->setText(QCoreApplication::translate("Analyse", "Hvis Charuco", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Analyse: public Ui_Analyse {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ANALYSE_H
