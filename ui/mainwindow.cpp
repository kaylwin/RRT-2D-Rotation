#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QTimerEvent>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    helper = new Helper();
    glWidget = new GLWidget(helper, ui->widget);


    // Set up rendering timer
    qtimer = new QTimer();
    qtimer->start(16);
    connect(qtimer, &QTimer::timeout, glWidget, &GLWidget::animate);

    // Initialize gravity vectors
    ui->verticalSlider_2->setSliderPosition(50);
    ui->verticalSlider->setSliderPosition(50); // Y should be negative
    connect(ui->verticalSlider_2, &QSlider::sliderMoved, glWidget, &GLWidget::x_update);
    connect(ui->verticalSlider, &QSlider::sliderMoved, glWidget, &GLWidget::y_update);

}

MainWindow::~MainWindow()
{
    delete ui;
    delete qtimer;
}

void MainWindow::on_verticalSlider_sliderReleased()
{
    //ui->lcdNumber->display(ui->verticalSlider->value());
}

void MainWindow::on_verticalSlider_valueChanged(int value)
{
    ui->lcdNumber->display(value);
}
