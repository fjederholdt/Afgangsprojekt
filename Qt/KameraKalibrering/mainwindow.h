#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtGui>
#include <QWidget>
#include <QMessageBox>
#include <QInputDialog>
#include <QFileDialog>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

static bool camera = false;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_Kalibrering_clicked();

    void on_Analyse_clicked();

    void on_cameraButton_clicked();

    void on_robotButton_clicked();

private:
    Ui::MainWindow *ui;
    QPixmap pix;
    QString path;

};
#endif // MAINWINDOW_H
