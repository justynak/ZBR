#include "simulationcontroller.h"

SimulationController::SimulationController(KinematicPoints *kinematics, TrajectoryPoints *trajectory, QObject *parent)
    : QObject(parent), kinematics(kinematics), trajectory(trajectory)
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(Tick()));
    // the fault can fire outside Running too, e.g. a Settings change that
    // makes the current pose unreachable
    connect(kinematics, SIGNAL(outOfRange()), this, SLOT(OnOutOfRange()));
}

void SimulationController::EnterState(State s)
{
    if(state == s) return;
    state = s;
    emit stateChanged(s);
}

void SimulationController::Start()
{
    if(state != Idle || trajectory->pointsNumber() == 0) return;
    EnterState(Running);
    // advance synchronously: the user gets progress feedback on the click
    // itself, and a path that faults right away shows the fault right away
    // instead of after a silent first interval
    AdvanceOnePoint();
    if(state == Running)
        timer.start();
}

void SimulationController::Stop()
{
    if(state != Running) return;
    timer.stop();
    EnterState(Idle); // currentPoint keeps its value: Start resumes
}

void SimulationController::Step()
{
    if(state == OutOfRange) return;
    if(state == Running)
    {
        timer.stop();
        EnterState(Idle);
    }
    if(trajectory->pointsNumber() == 0) return;
    AdvanceOnePoint();
}

void SimulationController::Tick()
{
    AdvanceOnePoint();
}

void SimulationController::AdvanceOnePoint()
{
    if(currentPoint >= trajectory->pointsNumber())
    {
        if(state == Running)
        {
            timer.stop();
            EnterState(Idle);
        }
        DropPath();
        emit progressChanged(100);
        return;
    }

    kinematics->CalculateMachineCoordinates((*trajectory)[currentPoint]);
    if(state == OutOfRange) return; // fault latched synchronously by OnOutOfRange

    ++currentPoint;
    emit progressChanged(100 * currentPoint / trajectory->pointsNumber());
}

void SimulationController::OnOutOfRange()
{
    timer.stop();
    EnterState(OutOfRange);
}

void SimulationController::ClearPath()
{
    if(state == OutOfRange) return; // only Reset leaves the fault state
    if(state == Running)
    {
        timer.stop();
        EnterState(Idle);
    }
    DropPath();
    emit progressChanged(0);
}

void SimulationController::Reset()
{
    timer.stop();
    DropPath();
    emit progressChanged(0);
    EnterState(Idle);
}

bool SimulationController::AddLinePath(QVector3D stop, int n)
{
    if(state == OutOfRange) return false;
    trajectory->GenerateLine(stop, n);
    return true;
}

bool SimulationController::AddCurvePath(QVector3D center, double fi, double theta, int n)
{
    if(state == OutOfRange) return false;
    trajectory->GenerateCurve(center, fi, theta, n);
    return true;
}

void SimulationController::DropPath()
{
    trajectory->ClearPoints(false);
    trajectory->SetNewTCP(kinematics->GetLastValidPoint());
    currentPoint = 0;
    emit pathChanged();
}
