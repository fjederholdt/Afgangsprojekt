#ifndef KALIBRERING_H
#define KALIBRERING_H

#include <QDialog>
#include <QMessageBox>
#include <filesystem>
#include <string>
#include <vector>
#include <time.h>

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

private slots:
    void on_ny_kalibrering_clicked();

    void on_annuller_clicked();

    void on_pushButton_clicked();

private:
    Ui::Kalibrering *ui;
    std::string path;
};

#endif // KALIBRERING_H
