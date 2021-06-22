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
    QPushButton *pushButton;

    void setupUi(QDialog *Analyse)
    {
        if (Analyse->objectName().isEmpty())
            Analyse->setObjectName(QString::fromUtf8("Analyse"));
        Analyse->resize(667, 489);
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
        pushButton = new QPushButton(Analyse);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(370, 330, 141, 21));

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
        pushButton->setText(QCoreApplication::translate("Analyse", "Hvis Charuco", nullptr));
    } // retranslateUi

};

namespace Ui {
    class Analyse: public Ui_Analyse {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ANALYSE_H
