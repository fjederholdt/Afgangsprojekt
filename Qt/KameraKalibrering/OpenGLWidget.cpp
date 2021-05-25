#include "OpenGLWidget.h"
#include <QMouseEvent>
#include <QOpenGLPaintDevice>
#include <QOpenGLContext>
#include <QPainter>
#include <QCoreApplication>
#include <math.h>
#include <iostream>
#include "mainwindow.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

GLWidget::~GLWidget()
{

}

unsigned char* cvMat2TexInput( Mat& img )
{

    cvtColor( img, img, COLOR_BGR2RGB );
    transpose( img, img );
    flip( img, img, 1 );
    flip( img, img, 0 );
    return img.data;

}

void GLWidget::initializeGL()
{
    glClearColor(0,0,0,1);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_COLOR_MATERIAL);
}

void GLWidget::paintGL()
{
   // QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    //f->glClear(GL_COLOR_BUFFER_BIT);
    glMapBuffer


}

