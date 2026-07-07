#ifndef SIMULATIONCONTROLLER_H
#define SIMULATIONCONTROLLER_H

#include <QObject>
#include <QTimer>
#include <QVector3D>

#include "kinematicpoints.h"
#include "trajectorypoints.h"

// Owns the animation state machine. Invariants:
// - the timer runs iff the state is Running
// - currentPoint is only meaningful while a path exists; 0 otherwise
// - whenever the path is dropped, the trajectory TCP is synced to the
//   robot's actual pose (KinematicPoints::GetLastValidPoint), so new
//   paths always start where the robot really is
//
// Event/state table:
//            Idle                 Running               OutOfRange
// start      run if path exists   ignored               ignored
// stop       ignored              pause -> Idle         ignored
// step       advance one point    pause + advance       ignored
// tick       (timer not running)  advance one point     (timer not running)
// clear      drop path            stop + drop -> Idle   ignored
// fault      -> OutOfRange        stop -> OutOfRange    latched
// addPath    allowed              allowed               rejected
// reset      drop path            stop + drop -> Idle   drop -> Idle
class SimulationController : public QObject
{
    Q_OBJECT

public:
    enum State { Idle, Running, OutOfRange };

    SimulationController(KinematicPoints *kinematics, TrajectoryPoints *trajectory, QObject *parent = 0);

    State GetState() const { return state; }

public slots:
    void Start();
    void Stop();
    void Step();
    void ClearPath();
    void Reset();
    void SetInterval(int ms) { timer.setInterval(ms); }

    bool AddLinePath(QVector3D stop, int n);
    bool AddCurvePath(QVector3D center, double fi, double theta, int n);

signals:
    void stateChanged(int state);
    void progressChanged(int percent);
    void pathChanged();
    // a newly added path reached outside the workspace and was cut at the
    // last reachable point; informational, not a fault
    void pathTrimmed(QVector3D lastReachable);

private slots:
    void Tick();
    void OnOutOfRange();
    void OnGeometryChanged();

private:
    void AdvanceOnePoint();
    void DropPath();
    void EnterState(State s);
    void ValidateNewPoints(int from);

    KinematicPoints *kinematics;
    TrajectoryPoints *trajectory;
    QTimer timer;
    State state = Idle;
    int currentPoint = 0;
};

#endif // SIMULATIONCONTROLLER_H
