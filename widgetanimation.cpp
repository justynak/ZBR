#include "widgetanimation.h"
#include "ui_widgetanimation.h"

#include <QGraphicsEllipseItem>

#define A -10
#define B 1000


/*
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
  */
WidgetAnimation::WidgetAnimation(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WidgetAnimation)
{
    ui->setupUi(this);
    sceneXY = new QGraphicsScene(this);
    sceneXZ = new QGraphicsScene(this);
    sceneYZ = new QGraphicsScene(this);

    finished = false;
    pathChanged = false;
    timer = new QTimer();
    currentPoint = 0;

    ui->graphicsViewXY->setScene(sceneXY);
    ui->graphicsViewXZ->setScene(sceneXZ);
    ui->graphicsViewYZ->setScene(sceneYZ);

    ui->graphicsViewXY->setRenderHint(QPainter::Antialiasing);
    ui->graphicsViewYZ->setRenderHint(QPainter::Antialiasing);
    ui->graphicsViewXZ->setRenderHint(QPainter::Antialiasing);

    ui->progressBar->setValue(0);


    //connect slots
    connect(ui->buttonCircleAdd, SIGNAL(clicked()), this, SLOT(on_buttonCircleAdd_clicked()));
    connect(ui->buttonClear, SIGNAL(clicked()), this, SLOT(on_buttonClear_clicked()));
    connect(ui->buttonLineAdd, SIGNAL(clicked()), this, SLOT(on_buttonLineAdd_clicked()));
    connect(ui->buttonStart, SIGNAL(clicked()), this, SLOT(on_buttonStart_clicked()));
    connect(ui->buttonStep, SIGNAL(clicked()), this, SLOT(on_buttonStep_clicked()));
    connect(ui->buttonStop, SIGNAL(clicked()), this, SLOT(on_buttonStop_clicked()));
    connect(ui->buttonStart, SIGNAL(clicked()), timer, SLOT(start()));
    connect(ui->buttonReset, SIGNAL(clicked()), this, SLOT(on_buttonReset_clicked()));
    connect(ui->sliderScale, SIGNAL(valueChanged(int)), this, SLOT(PaintJoints()));

    connect(timer, SIGNAL(timeout()), this, SLOT(GoToNextTrajectoryPoint()));

}

void WidgetAnimation::PaintGrid(double scale)
{
    QPen pen;
    pen.setWidth(1);
    pen.setColor(Qt::gray);

    for(int i=-20; i<21; ++i)
    {
        sceneXY->addLine(i*50*scale, 1000, i*50*scale, -1000, pen);
        sceneXY->addLine(1000, i*50*scale, -1000, i*50*scale, pen);

        sceneXZ->addLine(i*50*scale, 1000, i*50*scale, -1000, pen);
        sceneXZ->addLine(1000, i*50*scale, -1000, i*50*scale, pen);

        sceneYZ->addLine(i*50*scale, 1000, i*50*scale, -1000, pen);
        sceneYZ->addLine(1000, i*50*scale, -1000, i*50*scale, pen);
    }

}


WidgetAnimation::~WidgetAnimation()
{
    delete ui;
}

void WidgetAnimation::StartSimuation()
{
    currentPoint = 0;
    ChangeSpeed();
}

void WidgetAnimation::ClearPath()
{
    trajectory->ClearPoints(finished);
    sceneXY->clear();
    sceneXZ->clear();
    sceneYZ->clear();
    PaintJoints();

    ui->progressBar->setValue(0);
}

void WidgetAnimation::RemovePath()
{
    PaintJoints();
    trajectory->ClearPoints(true);
}

void WidgetAnimation::OutOfRangeError()
{
    QPixmap pix = QPixmap(70, 50);
    pix.fill(Qt::darkRed);
    ui->labelValid->setPixmap(pix);

    ClearPath();
    timer->stop();
    jointPoints->CalculateMachineCoordinates(jointPoints->GetLastValidPoint());
    PaintJoints();
    currentPoint = 0;
    trajectory->SetNewTCP(jointPoints->GetLastValidPoint());
}

void WidgetAnimation::GoToNextTrajectoryPoint()
{
    if(trajectory->pointsNumber() != 0 )
    {

        if(currentPoint >= trajectory->pointsNumber())
        {
            timer->stop();
            currentPoint = 0;
            finished = true;
            RemovePath();
            ui->progressBar->setValue(100);
        }

        else
        {
            QVector3D p = (*trajectory)[currentPoint];
            jointPoints->CalculateMachineCoordinates(p);

            if(trajectory->pointsNumber())
                ui->progressBar->setValue(100 * (currentPoint+1) / trajectory->pointsNumber());

            ++currentPoint;
        }
    }
}

void WidgetAnimation::CreateLinearPath()
{
    //get start, stop and nr of steps
    QVector3D stop = QVector3D(ui->lineEditStopX->text().toDouble(), ui->lineEditStopY->text().toDouble(), ui->lineEditStopZ->text().toDouble());
    int n = ui->lineEditNLine->text().toInt();
    trajectory->GenerateLine( stop, n);
}

void WidgetAnimation::CreateCurvePath()
{
    //get radius, center
    QVector3D center = QVector3D(ui->lineEditCenterX->text().toDouble(), ui->lineEditCenterY->text().toDouble(),  ui->lineEditCenterZ->text().toDouble());
    double fi = ui->lineEditFi->text().toDouble();
    double theta = ui->lineEditTheta->text().toDouble();
    int n = ui->lineEditNCircle->text().toInt();
    trajectory->GenerateCurve(center, fi, theta, n);
}

void WidgetAnimation::PaintJoints()
{
    QPen pen(Qt::black);
    pen.setWidth(5);
    QBrush brush(Qt::red);

    QList <QVector3D> pointList = jointPoints->GetJointPoints();

    qreal scale = ui->sliderScale->value() * (-0.018) + 1.9;

    sceneXY->clear();
    sceneXZ->clear();
    sceneYZ->clear();

    PaintGrid(scale);


    for(int i=0; i<pointList.size(); ++i)
    {
        pointList[i] =  pointList[i] * scale;
    }

    //XY - get x, y coordinates
    for(int i=0; i<pointList.size() - 1; ++i)
    {   
        sceneXY->addLine(pointList[i].x(), pointList[i].y(), pointList[i+1].x(), pointList[i+1].y(), pen );
        sceneXY->addEllipse(pointList[i].x()-5, pointList[i].y()-5, 10, 10, pen, brush);
        sceneXZ->addLine(pointList[i].x(), pointList[i].z(), pointList[i+1].x(),pointList[i+1].z(), pen );
        sceneXZ->addEllipse(pointList[i].x()-5, pointList[i].z()-5, 10, 10, pen, brush);
        sceneYZ->addLine(pointList[i].y()-5, pointList[i].z()-5, pointList[i+1].y(), pointList[i+1].z(), pen );
        sceneYZ->addEllipse(pointList[i].y()-5, pointList[i].z()-5, 10, 10, pen, brush);
    }

    int j = pointList.size() -1;
    sceneXY->addEllipse(pointList[j].x()-5, pointList[j].y()-5, 10, 10, pen, brush);
    sceneXZ->addEllipse(pointList[j].x()-5, pointList[j].z()-5, 10, 10, pen, brush);
    sceneYZ->addEllipse(pointList[j].y()-5, pointList[j].z()-5, 10, 10, pen, brush);

    pen.setWidth(1);
    pen.setColor(Qt::darkGray);

    //grid
    for(int i=-600; i>0; i+=100)
    {
        sceneXY->addLine(i, 0, -i, 0, pen);
        sceneXY->addLine(0, -i, 0, -i, pen);

        sceneXZ->addLine(i, 0, -i, 0, pen);
        sceneXZ->addLine(0, -i, 0, -i, pen);

        sceneYZ->addLine(i, 0,-i, 0, pen);
        sceneYZ->addLine(0, -i, 0, -i, pen);
    }

    PaintPath();
}

void WidgetAnimation::ChangeSpeed()
{
    int speed = ui->sliderSpeed->value();
    speed = ui->sliderSpeed->value() * A + B;
    timer->setInterval(speed);
}

void WidgetAnimation::PaintPath()
{
    QPen pen(Qt::red);
    pen.setWidth(1);
    QBrush brush(Qt::green);
    int max = trajectory->pointsNumber();
    QList<QVector3D> pointList;

    pointList = trajectory->GetPointList();

    qreal scale = ui->sliderScale->value() * (-0.018) + 1.9;
    for(int i=0; i<pointList.size(); ++i)
    {
        pointList[i] =  pointList[i] * scale;
    }


    for(int i=0; i<max-1; ++i)
    {
        sceneXY->addLine(pointList[i].x(), pointList[i].y(), pointList[i+1].x(), pointList[i+1].y(), pen );
        sceneXY->addEllipse(pointList[i].x()-2, pointList[i].y()-2, 4, 4, pen, brush);
        sceneXZ->addLine(pointList[i].x(), pointList[i].z(), pointList[i+1].x(),pointList[i+1].z(), pen );
        sceneXZ->addEllipse(pointList[i].x()-2, pointList[i].z()-2, 4, 4, pen, brush);
        sceneYZ->addLine(pointList[i].y(), pointList[i].z(), pointList[i+1].y(), pointList[i+1].z(), pen );
        sceneYZ->addEllipse(pointList[i].y()-2, pointList[i].z()-2, 4, 4, pen, brush);
    }

    if(max>=1)
    {
        sceneXY->addEllipse(pointList[max-1].x()-2, pointList[max-1].y()-2, 4, 4, pen, brush);
        sceneXZ->addEllipse(pointList[max-1].x()-2, pointList[max-1].z()-2, 4, 4, pen, brush);
        sceneYZ->addEllipse(pointList[max-1].y()-2, pointList[max-1].z()-2, 4, 4, pen, brush);
    }
    pathChanged = false;
}

void WidgetAnimation::SetStatusValid()
{

    QPixmap pix = QPixmap(70, 50);
    pix.fill(Qt::darkGreen);
    ui->labelValid->setPixmap(pix);
}

void WidgetAnimation::on_buttonClear_clicked()
{
    ClearPath();
}

void WidgetAnimation::on_buttonStep_clicked()
{
    timer->stop();
    GoToNextTrajectoryPoint();
}

void WidgetAnimation::on_buttonStop_clicked()
{
    timer->stop();
}

void WidgetAnimation::on_buttonStart_clicked()
{
    ChangeSpeed();
    timer->start();
    finished = false;
}

void WidgetAnimation::on_buttonCircleAdd_clicked()
{
    static bool clicked = 0;
    if(!clicked)
    {
        CreateCurvePath();
    }
    clicked = !clicked;

}
void WidgetAnimation::on_buttonLineAdd_clicked()
{    
    static bool clicked = 0;

    if(!clicked)
    {
        CreateLinearPath();
    }
    clicked = !clicked;

}

void WidgetAnimation::on_sliderSpeed_sliderMoved(int position)
{
    ChangeSpeed();
}

void WidgetAnimation::on_buttonReset_clicked()
{
    jointPoints->RestoreCustomSettings();
    trajectory->SetNewTCP(jointPoints->GetToolPoint());
}
