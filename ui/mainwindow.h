#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "glwidget.h"
#include "helper.h"
#include "QTimer"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_verticalSlider_sliderReleased();

    void on_verticalSlider_valueChanged(int value);

private:
    Ui::MainWindow *ui;
    Helper* helper;
    GLWidget* glWidget;
    QTimer* qtimer;
};

#endif // MAINWINDOW_H
