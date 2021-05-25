#ifndef OPENGLWIDGET_H
#define OPENGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QMatrix4x4>

class GLWidget : public QOpenGLWidget
{
public:
    GLWidget(QWidget *parent = 0) : QOpenGLWidget(parent){};
    ~GLWidget();

protected:
    void initializeGL();
    void paintGL();
    //void resizeGL(int width, int height) override;
};

#endif // OPENGLWIDGET_H
