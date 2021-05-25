#ifndef ANALYSE_H
#define ANALYSE_H

#include <QDialog>
#include <string>

namespace Ui {
class Analyse;
}

class Analyse : public QDialog
{
    Q_OBJECT

public:
    explicit Analyse(QWidget *parent = nullptr);
    ~Analyse();
    void setValg(const QString &path);
    QString getValg();
private:
    QString valgteKalib;
    Ui::Analyse *ui;
};

#endif // ANALYSE_H
