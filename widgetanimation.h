#ifndef WIDGETANIMATION_H
#define WIDGETANIMATION_H

#include <QWidget>
#include <QGraphicsScene>
#include <QtGui>
#include <QVector3D>

#include "trajectorypoints.h"
#include "kinematicpoints.h"

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
        connect(jointPoints, SIGNAL(outOfRange()), this, SLOT(OutOfRangeError()));
        connect(jointPoints, SIGNAL(statusOK()), this, SLOT(PaintJoints()));
        connect(jointPoints, SIGNAL(statusOK()), this, SLOT(SetStatusValid()));
    }
    void SetTrajectoryPoints(TrajectoryPoints *p) {
        trajectory= p;
        connect(trajectory, SIGNAL(pathGenerated()), this, SLOT(PaintPath()));
    }

    void PaintGrid(double scale);

    ~WidgetAnimation();

private:
      Ui::WidgetAnimation * ui;
      QGraphicsScene* sceneXY;
      QGraphicsScene* sceneXZ;
      QGraphicsScene* sceneYZ;
      QTimer *timer;

      KinematicPoints* jointPoints;
      TrajectoryPoints* trajectory;

      int currentPoint;
      bool finished;
      bool pathChanged;

signals:
    
public slots:
      void StartSimuation();
      void ClearPath();
      void RemovePath();

      void OutOfRangeError();
      void GoToNextTrajectoryPoint();
      void CreateLinearPath();
      void CreateCurvePath();
      void PaintJoints();
      void ChangeSpeed();
      void PaintPath();
      void SetStatusValid();


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
