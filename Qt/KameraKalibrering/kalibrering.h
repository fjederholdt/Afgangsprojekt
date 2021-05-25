#ifndef KALIBRERING_H
#define KALIBRERING_H

#include <QDialog>
#include <QMessageBox>

namespace Ui {
class Kalibrering;
}

class Kalibrering : public QDialog
{
    Q_OBJECT

public:
    explicit Kalibrering(QWidget *parent = nullptr);
    ~Kalibrering();
private slots:
    void on_ny_kalibrering_clicked();

    void on_valg_kalibrering_clicked();

private:
    Ui::Kalibrering *ui;
    std::string path;
};

#endif // KALIBRERING_H
