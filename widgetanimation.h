#ifndef WIDGETANIMATION_H
#define WIDGETANIMATION_H

#include <QWidget>
#include <QGraphicsScene>
#include <QtWidgets>
#include <QVector3D>

#include "trajectorypoints.h"
#include "kinematicpoints.h"
#include "simulationcontroller.h"

namespace Ui {
    class WidgetAnimation;
}



class WidgetAnimation : public QWidget
{
    Q_OBJECT
public:
    explicit WidgetAnimation(QWidget *parent = 0);
    void SetKinematicPoints(KinematicPoints *p) {
        jointPoints = p;
        connect(jointPoints, SIGNAL(statusOK()), this, SLOT(PaintJoints()));
    }
    void SetTrajectoryPoints(TrajectoryPoints *p) {
        trajectory= p;
        connect(trajectory, SIGNAL(pathGenerated()), this, SLOT(PaintPath()));
    }
    void SetController(SimulationController *c) {
        controller = c;
        connect(controller, SIGNAL(stateChanged(int)), this, SLOT(OnStateChanged(int)));
        connect(controller, SIGNAL(progressChanged(int)), this, SLOT(OnProgressChanged(int)));
        connect(controller, SIGNAL(pathChanged()), this, SLOT(PaintJoints()));
        connect(controller, SIGNAL(pathTrimmed(QVector3D)), this, SLOT(OnPathTrimmed(QVector3D)));
    }

    void PaintGrid(double scale);

    ~WidgetAnimation();

private:
      int IntervalMs() const;

      Ui::WidgetAnimation * ui;
      QGraphicsScene* sceneXY;
      QGraphicsScene* sceneXZ;
      QGraphicsScene* sceneYZ;

      KinematicPoints* jointPoints;
      TrajectoryPoints* trajectory;
      SimulationController* controller;

signals:

public slots:
      void PaintJoints();
      void PaintPath();
      void OnStateChanged(int state);
      void OnProgressChanged(int percent);
      void OnPathTrimmed(QVector3D lastReachable);

private slots:
      void on_buttonClear_clicked();
      void on_buttonStep_clicked();
      void on_buttonStop_clicked();
      void on_buttonStart_clicked();
      void on_buttonCircleAdd_clicked();
      void on_buttonLineAdd_clicked();
      void on_sliderSpeed_sliderMoved(int position);
      void on_buttonReset_clicked();
};

#endif // WIDGETANIMATION_H
