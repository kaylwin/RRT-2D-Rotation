#ifndef HELPER_H
#define HELPER_H
#include <QBrush>
#include <QFont>
#include <QPen>
#include <QPainter>
#include <QPaintEvent>

struct UiCommands {
    float gravity_x = 0.0f;
    float gravity_y = 0.0f;
};
class Helper
{
public:
    Helper();
    virtual ~Helper();
    UiCommands* commands;
public:
    void paint(QPainter *painter, QPaintEvent *event, int elapsed);

private:
    QBrush background;
    QBrush circleBrush;
    QFont textFont;
    QPen circlePen;
    QPen textPen;

};
#endif // HELPER_H
