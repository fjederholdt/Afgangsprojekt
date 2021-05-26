/********************************************************************************
** Form generated from reading UI file 'nykalibrering.ui'
**
** Created by: Qt User Interface Compiler version 5.15.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_NYKALIBRERING_H
#define UI_NYKALIBRERING_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_NyKalibrering
{
public:
    QPushButton *tag_billede;
    QPushButton *valg_alle_billeder;
    QPushButton *slet_billede;
    QPushButton *brug_vs;
    QPushButton *kalibrere;
    QPushButton *auto_billede;
    QListWidget *listWidget;
    QLabel *label;
    QPushButton *annuller;

    void setupUi(QDialog *NyKalibrering)
    {
        if (NyKalibrering->objectName().isEmpty())
            NyKalibrering->setObjectName(QString::fromUtf8("NyKalibrering"));
        NyKalibrering->resize(732, 520);
        tag_billede = new QPushButton(NyKalibrering);
        tag_billede->setObjectName(QString::fromUtf8("tag_billede"));
        tag_billede->setGeometry(QRect(360, 20, 89, 25));
        valg_alle_billeder = new QPushButton(NyKalibrering);
        valg_alle_billeder->setObjectName(QString::fromUtf8("valg_alle_billeder"));
        valg_alle_billeder->setGeometry(QRect(330, 130, 161, 25));
        slet_billede = new QPushButton(NyKalibrering);
        slet_billede->setObjectName(QString::fromUtf8("slet_billede"));
        slet_billede->setGeometry(QRect(350, 70, 121, 25));
        brug_vs = new QPushButton(NyKalibrering);
        brug_vs->setObjectName(QString::fromUtf8("brug_vs"));
        brug_vs->setGeometry(QRect(380, 280, 89, 25));
        kalibrere = new QPushButton(NyKalibrering);
        kalibrere->setObjectName(QString::fromUtf8("kalibrere"));
        kalibrere->setGeometry(QRect(470, 410, 89, 25));
        auto_billede = new QPushButton(NyKalibrering);
        auto_billede->setObjectName(QString::fromUtf8("auto_billede"));
        auto_billede->setGeometry(QRect(380, 340, 211, 25));
        listWidget = new QListWidget(NyKalibrering);
        listWidget->setObjectName(QString::fromUtf8("listWidget"));
        listWidget->setGeometry(QRect(30, 20, 256, 192));
        label = new QLabel(NyKalibrering);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(50, 230, 221, 17));
        annuller = new QPushButton(NyKalibrering);
        annuller->setObjectName(QString::fromUtf8("annuller"));
        annuller->setGeometry(QRect(70, 380, 89, 25));

        retranslateUi(NyKalibrering);

        QMetaObject::connectSlotsByName(NyKalibrering);
    } // setupUi

    void retranslateUi(QDialog *NyKalibrering)
    {
        NyKalibrering->setWindowTitle(QCoreApplication::translate("NyKalibrering", "Dialog", nullptr));
        tag_billede->setText(QCoreApplication::translate("NyKalibrering", "Tag billede", nullptr));
        valg_alle_billeder->setText(QCoreApplication::translate("NyKalibrering", "v\303\246lg alle billeder", nullptr));
        slet_billede->setText(QCoreApplication::translate("NyKalibrering", "slet billede(r)", nullptr));
        brug_vs->setText(QCoreApplication::translate("NyKalibrering", "Brug VS", nullptr));
        kalibrere->setText(QCoreApplication::translate("NyKalibrering", "Kalibrere", nullptr));
        auto_billede->setText(QCoreApplication::translate("NyKalibrering", "automatisk billedetagning", nullptr));
        label->setText(QCoreApplication::translate("NyKalibrering", "Billede nr. linker til robot pose", nullptr));
        annuller->setText(QCoreApplication::translate("NyKalibrering", "Annuller", nullptr));
    } // retranslateUi

};

namespace Ui {
    class NyKalibrering: public Ui_NyKalibrering {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_NYKALIBRERING_H
