#ifndef ANALYSE_H
#define ANALYSE_H

#include <QDialog>
#include <QDebug>
#include <QMessageBox>
#include <QApplication>
#include <QHeaderView>
#include <QFormLayout>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <time.h>

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

private:
    Ui::Analyse *ui;
    std::string path;
};

#endif // ANALYSE_H
