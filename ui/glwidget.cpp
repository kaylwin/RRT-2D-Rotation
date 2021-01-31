#include <iostream>
#include <QTimer>
#include "glwidget.h"

GLWidget::GLWidget(Helper *helper, QWidget *parent)
    : QOpenGLWidget(parent), helper(helper)
{
    elapsed = 0;
    setAutoFillBackground(false);
    setFixedSize(parent->width(), parent->height());
    helper->commands = &this->commands;
}

void GLWidget::animate()
{
    elapsed = (elapsed + qobject_cast<QTimer*>(sender())->interval()) % 1000;
    update();
}
void GLWidget::y_update(const int& value){
   this->commands.gravity_y = static_cast<float>(value - 50) / 50.0f * 10;
}

void GLWidget::x_update(const int& value){
    this->commands.gravity_x = static_cast<float>(value - 50) / 50.0f * 10;
}
void GLWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter;
    painter.begin(this);
    painter.setRenderHint(QPainter::Antialiasing);
    helper->paint(&painter, event, elapsed);
    painter.end();
}
