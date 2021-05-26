#include "analyse.h"
#include "ui_analyse.h"

using namespace std::filesystem;

Analyse::Analyse(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Analyse)
{
    ui->setupUi(this);
    ui->tableWidget->setColumnCount(3);
    QStringList tableHeader;
    tableHeader << "Dato" << "Antal Billeder" << "Rep Error";
    ui->tableWidget->setHorizontalHeaderLabels(tableHeader);
    ui->tableWidget->horizontalHeader()->setStretchLastSection(true);
    ui->tableWidget->verticalHeader()->setVisible(false);
    ui->tableWidget->setSelectionBehavior(QAbstractItemView::SelectRows);
    ui->tableWidget->setShowGrid(false);
    path = "/home/jeppe/Qt-workspace/KameraKalibrering/Kalibreringer/";
    std::vector<std::string> pathVector;
    std::string sub, suffix;
    int maxCols = 3;
    for(const auto & entry : directory_iterator(path))
    {
        sub = entry.path();
        std::size_t found = sub.find_last_of("/");
        sub = sub.substr(found+1,sub.size());
        pathVector.push_back(sub);
    }
    ui->tableWidget->setRowCount(pathVector.size());
    std::vector<std::string> billedePath;
    for (size_t i = 0; i < pathVector.size(); i++)
    {
        for(const auto & entry : directory_iterator(path+pathVector.at(i)))
        {
            sub = entry.path();
            std::size_t found = sub.find_last_of("/");
            sub = sub.substr(found+1,sub.size());
            billedePath.push_back(sub);
        }
        for(int currentCol = 0; currentCol < maxCols; currentCol++)
        {
            switch(currentCol){
                case 0:
                {
                    QTableWidgetItem *item = new QTableWidgetItem();
                    item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                    item->setText(QString::fromStdString(pathVector.at(i)));
                    ui->tableWidget->setItem(i,0,item);
                    break;
                }
                case 1:
                {
                    QTableWidgetItem *item = new QTableWidgetItem();
                    item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                    item->setText(QString::fromStdString(std::to_string(billedePath.size())));
                    item->setTextAlignment(Qt::AlignCenter);
                    ui->tableWidget->setItem(i,1,item);
                    break;
                }
                case 2:
                {
                    QTableWidgetItem *item = new QTableWidgetItem();
                    item->setFlags(item->flags() ^ Qt::ItemIsEditable);
                    item->setText(QString::fromStdString("0.00001"));
                    item->setTextAlignment(Qt::AlignRight);
                    ui->tableWidget->setItem(i,2,item);
                    break;
                }
            }
        }
        billedePath.clear();
    }
    ui->tableWidget->resizeColumnsToContents();
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Fixed);

}

Analyse::~Analyse()
{
    delete ui;
}

void Analyse::on_anvend_valgte_kalibrering_clicked()
{
    QItemSelectionModel * select = ui->tableWidget->selectionModel();
    QList<QModelIndex> rows = select->selectedRows();
    if(rows.size() == 1)
    {
        QModelIndex index = rows.at(0);
        int row = index.row();
        int col = 0;
        QString datoStr = ui->tableWidget->item(row,col)->text();
        QString kalibPath = QString::fromStdString(path)+datoStr;
        qDebug() << kalibPath;
    }
    else if(rows.size() > 1)
    {
        QMessageBox msg;
        msg.setText(QString::fromStdString("For mange kalibreringer valgt - vælg kun én kalibrering"));
        msg.exec();
    }
    else
    {
        QMessageBox msg;
        msg.setText(QString::fromStdString("Ingen kalibrering valgt - vælg kalibrering fra tabelen"));
        msg.exec();
    }

}

void Analyse::on_test_kalibrering_clicked()
{

}

void Analyse::on_visualiser_clicked()
{
    QItemSelectionModel * select = ui->tableWidget->selectionModel();
    QList<QModelIndex> rows = select->selectedRows();
    if(rows.size() == 1)
    {
        QModelIndex index = rows.at(0);
        int row = index.row();
        int col = 0;
        QString datoStr = ui->tableWidget->item(row,col)->text();
        QString kalibPath = QString::fromStdString(path)+datoStr;
        qDebug() << kalibPath;
    }
    else if(rows.size() > 1)
    {
        QMessageBox msg;
        msg.setText(QString::fromStdString("For mange kalibreringer valgt - vælg kun én kalibrering"));
        msg.exec();
    }
    else
    {
        QMessageBox msg;
        msg.setText(QString::fromStdString("Ingen kalibrering valgt - vælg kalibrering fra tabelen"));
        msg.exec();
    }
}
