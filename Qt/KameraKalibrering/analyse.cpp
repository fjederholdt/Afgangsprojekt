#include "analyse.h"
#include "ui_analyse.h"

Analyse::Analyse(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Analyse)
{
    ui->setupUi(this);
}

Analyse::~Analyse()
{
    delete ui;
}

void Analyse::setValg(const QString &path)
{
    valgteKalib = path;
}

QString Analyse::getValg()
{
    return valgteKalib;
}
