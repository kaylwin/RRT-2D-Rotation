#ifndef GLWIDGET_H
#define GLWIDGET_H

#include "helper.h"
#include <QOpenGLWidget>
#include <QObject>
class Helper;


class GLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    GLWidget(Helper *helper, QWidget *parent);

    UiCommands commands;

public slots:
    void animate();
    void y_update(const int& value);
    void x_update(const int& value);
    void g_update(const int& value);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    Helper *helper;
    int elapsed;

};
#endif // GLWIDGET_H
