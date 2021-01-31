#ifndef WIDGET_H
#define WIDGET_H
#include <QWidget>
#include <QPainter>
#include "helper.h"


class Helper;

class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(Helper *helper, QWidget *parent);

public slots:
    void animate();

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    Helper *helper;
    int elapsed;
};

#endif // WIDGET_H
