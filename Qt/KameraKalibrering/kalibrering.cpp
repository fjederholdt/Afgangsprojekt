#include "kalibrering.h"
#include "ui_kalibrering.h"
#include "nykalibrering.h"
#include "mainwindow.h"
#include <filesystem>
#include <string>
#include <vector>
#include <time.h>

using namespace std::filesystem;

Kalibrering::Kalibrering(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Kalibrering)
{
    ui->setupUi(this);
    std::vector<std::string> pathVector;
    std::string sub, suffix;
    path = "/home/jeppe/Qt-workspace/KameraKalibrering/Kalibreringer/";
    for(const auto & entry : directory_iterator(path))
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

Kalibrering::~Kalibrering()
{
    delete ui;
}

void Kalibrering::on_ny_kalibrering_clicked()
{
    time_t rawtime;
    time(&rawtime);
    std::string localTime = asctime(localtime(&rawtime));
    localTime.pop_back();
    NyKalibrering nyKalibrering;
    nyKalibrering.setModal(true);
    nyKalibrering.setWindowTitle(QString::fromStdString("Kalibrering: "+ localTime));
    nyKalibrering.exec();
}

void Kalibrering::on_valg_kalibrering_clicked()
{
    QListWidgetItem *item =ui->listWidget->currentItem();

    MainWindow::setValg(QString::fromStdString(path)+item->text());
    Kalibrering::close();
}

