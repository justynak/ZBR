#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include "widgetanimation.h"
#include "trajectorypoints.h"
#include "kinematicpoints.h"
#include "simulationcontroller.h"
#include "widgetsettings.h"
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
private:
    Ui::MainWindow *ui;

    //instances
    TrajectoryPoints* trajectory;
    KinematicPoints* jointPoints;
    SimulationController* controller;

    WidgetAnimation* animation;
    WidgetSettings* settings;


signals:
    void dimensionChanged(int n, int dim);
    void simulationStopped();

public slots:

};

#endif // MAINWINDOW_H
