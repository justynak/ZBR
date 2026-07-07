#include "widgetanimation.h"
#include "ui_widgetanimation.h"

#include <QGraphicsEllipseItem>

#define A -10
#define B 1000


WidgetAnimation::WidgetAnimation(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WidgetAnimation)
{
    ui->setupUi(this);
    sceneXY = new QGraphicsScene(this);
    sceneXZ = new QGraphicsScene(this);
    sceneYZ = new QGraphicsScene(this);

    ui->graphicsViewXY->setScene(sceneXY);
    ui->graphicsViewXZ->setScene(sceneXZ);
    ui->graphicsViewYZ->setScene(sceneYZ);

    ui->graphicsViewXY->setRenderHint(QPainter::Antialiasing);
    ui->graphicsViewYZ->setRenderHint(QPainter::Antialiasing);
    ui->graphicsViewXZ->setRenderHint(QPainter::Antialiasing);

    ui->progressBar->setValue(0);
    OnStateChanged(SimulationController::Idle);

    // the on_<object>_<signal> slots are auto-connected by
    // connectSlotsByName inside setupUi — do not connect them again here,
    // or every click fires twice
    connect(ui->sliderScale, SIGNAL(valueChanged(int)), this, SLOT(PaintJoints()));
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

int WidgetAnimation::IntervalMs() const
{
    return ui->sliderSpeed->value() * A + B;
}

void WidgetAnimation::OnStateChanged(int state)
{
    QPixmap pix = QPixmap(70, 50);
    pix.fill(state == SimulationController::OutOfRange ? Qt::darkRed : Qt::darkGreen);
    ui->labelValid->setPixmap(pix);
}

void WidgetAnimation::OnProgressChanged(int percent)
{
    ui->progressBar->setValue(percent);
}

void WidgetAnimation::OnPathTrimmed(QVector3D lastReachable)
{
    // non-modal so it never blocks the simulation (or a test)
    QMessageBox *box = new QMessageBox(QMessageBox::Warning, tr("Path trimmed"),
        tr("The full path is not reachable.\nIt was trimmed to (%1, %2, %3).")
            .arg(lastReachable.x()).arg(lastReachable.y()).arg(lastReachable.z()),
        QMessageBox::Ok, this);
    box->setAttribute(Qt::WA_DeleteOnClose);
    box->show();
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
}

void WidgetAnimation::on_buttonClear_clicked()
{
    controller->ClearPath();
}

void WidgetAnimation::on_buttonStep_clicked()
{
    controller->Step();
}

void WidgetAnimation::on_buttonStop_clicked()
{
    controller->Stop();
}

void WidgetAnimation::on_buttonStart_clicked()
{
    controller->SetInterval(IntervalMs());
    controller->Start();
}

void WidgetAnimation::on_buttonCircleAdd_clicked()
{
    QVector3D center = QVector3D(ui->lineEditCenterX->text().toDouble(), ui->lineEditCenterY->text().toDouble(),  ui->lineEditCenterZ->text().toDouble());
    double fi = ui->lineEditFi->text().toDouble();
    double theta = ui->lineEditTheta->text().toDouble();
    int n = ui->lineEditNCircle->text().toInt();
    controller->AddCurvePath(center, fi, theta, n);
}

void WidgetAnimation::on_buttonLineAdd_clicked()
{
    QVector3D stop = QVector3D(ui->lineEditStopX->text().toDouble(), ui->lineEditStopY->text().toDouble(), ui->lineEditStopZ->text().toDouble());
    int n = ui->lineEditNLine->text().toInt();
    controller->AddLinePath(stop, n);
}

void WidgetAnimation::on_sliderSpeed_sliderMoved(int position)
{
    controller->SetInterval(IntervalMs());
}

void WidgetAnimation::on_buttonReset_clicked()
{
    jointPoints->RestoreCustomSettings();
    controller->Reset();
}
