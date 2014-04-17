#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    currentPoint=0;
    jointPoints = new KinematicPoints();
    trajectory = new TrajectoryPoints(jointPoints->GetToolPoint());


    ui->setupUi(this);

    animation = new WidgetAnimation();
    animation->SetTrajectoryPoints(trajectory);
    animation->SetKinematicPoints(jointPoints);

    settings = new WidgetSettings();
    settings->SetKinematicPoints(jointPoints);

    jointPoints->Initialize();

    settings->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    settings->resize(ui->scrollArea2->size().width() ,ui->scrollArea2->size().height());
    ui->scrollArea2->setWidgetResizable(true);
    ui->scrollArea2->setWidget(settings);
    settings->adjustSize();

    animation->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    animation->resize(ui->scrollArea->size().width() ,ui->scrollArea->size().height());
    ui->scrollArea->setWidgetResizable(true);
    ui->scrollArea->setWidget(animation);
    animation->adjustSize();

    ui->tabs->setCurrentIndex(0);
    ui->tabs->setTabText(0, "Animation");
    ui->tabs->setTabText(1, "Settings");

}

MainWindow::~MainWindow()
{
    delete ui;
}



