#ifndef KALIBRERING_H
#define KALIBRERING_H

#include <QDialog>
#include <QMessageBox>
#include <filesystem>
#include <set>
#include <string>
#include <vector>
#include <time.h>

#include "fsclass.h"
#include "dhparams.h"

namespace Ui {
class Kalibrering;
}

class Kalibrering : public QDialog
{
    Q_OBJECT

public:
    explicit Kalibrering(QWidget *parent = nullptr);
    ~Kalibrering();
    void removeKalib();
    void setPath(const QString &mainPath);
    QString getPath();

private slots:
    void on_ny_kalibrering_clicked();

    void on_annuller_clicked();

    void on_slet_kalibrering_clicked();

    void on_kalibrere_clicked();

private:
    Ui::Kalibrering *ui;
    std::string folderpath;
};

#endif // KALIBRERING_H
