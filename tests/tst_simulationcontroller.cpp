#include <QtTest>
#include <QSignalSpy>
#include <QVector3D>

#include "simulationcontroller.h"

// Tests for the animation state machine (Idle / Running / OutOfRange).
// The controller owns the timer, the path cursor, and the invariant that
// dropping a path syncs the trajectory TCP to the robot's actual pose.

class TestSimulationController : public QObject
{
    Q_OBJECT

private slots:
    void startWithoutPathIsIgnored();
    void runsPathToCompletionAndReturnsToIdle();
    void stepAdvancesExactlyOnePoint();
    void stepPausesARunningAnimation();
    void clearWhileRunningStopsEverything();
    void newPathAfterClearStartsFromRobotPose();
    void outOfRangeLatchesUntilReset();
    void faultCanFireFromIdle();
};

void TestSimulationController::startWithoutPathIsIgnored()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);

    ctrl.Start();

    QCOMPARE(ctrl.GetState(), SimulationController::Idle);
}

void TestSimulationController::runsPathToCompletionAndReturnsToIdle()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);
    QSignalSpy progress(&ctrl, SIGNAL(progressChanged(int)));

    QVector3D stop(500, 200, 300);
    tp.GenerateLine(stop, 4);

    ctrl.SetInterval(0);
    ctrl.Start();
    QCOMPARE(ctrl.GetState(), SimulationController::Running);

    QTRY_COMPARE(ctrl.GetState(), SimulationController::Idle);

    QCOMPARE(kp.GetToolPoint(), stop);          // robot walked the whole path
    QCOMPARE(tp.pointsNumber(), 0);             // path dropped on completion
    QCOMPARE(progress.last().first().toInt(), 100);
    // TCP synced to the robot: the next path starts at the stop point
    tp.GenerateLine(QVector3D(500, 300, 300), 1);
    QCOMPARE(tp[0], stop);
}

void TestSimulationController::stepAdvancesExactlyOnePoint()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);

    tp.GenerateLine(QVector3D(500, 200, 300), 4);

    ctrl.Step();
    ctrl.Step();

    QCOMPARE(ctrl.GetState(), SimulationController::Idle);
    QCOMPARE(kp.GetToolPoint(), tp[1]);
}

void TestSimulationController::stepPausesARunningAnimation()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);

    tp.GenerateLine(QVector3D(500, 200, 300), 4);
    ctrl.SetInterval(60000); // no tick will fire on its own
    ctrl.Start();
    QCOMPARE(ctrl.GetState(), SimulationController::Running);

    ctrl.Step();

    QCOMPARE(ctrl.GetState(), SimulationController::Idle);
    QCOMPARE(kp.GetToolPoint(), tp[0]);
}

// The bug that motivated the state machine: Clear while Running used to
// leave a zombie timer, a stale path cursor, and a stale TCP behind.
void TestSimulationController::clearWhileRunningStopsEverything()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);
    QSignalSpy progress(&ctrl, SIGNAL(progressChanged(int)));

    tp.GenerateLine(QVector3D(500, 200, 300), 4);
    ctrl.SetInterval(60000);
    ctrl.Start();

    ctrl.ClearPath();

    QCOMPARE(ctrl.GetState(), SimulationController::Idle);
    QCOMPARE(tp.pointsNumber(), 0);
    QCOMPARE(progress.last().first().toInt(), 0);

    // the cursor was reset: a new path plays from its first point
    tp.GenerateLine(QVector3D(450, 250, 300), 4);
    ctrl.Step();
    QCOMPARE(kp.GetToolPoint(), tp[0]);
}

// Dropping a path must sync the TCP to where the robot actually is —
// not where the path started, and not where it would have ended.
void TestSimulationController::newPathAfterClearStartsFromRobotPose()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);

    tp.GenerateLine(QVector3D(500, 200, 300), 4);
    ctrl.Step();
    ctrl.Step(); // robot is now mid-path

    QVector3D pose = kp.GetLastValidPoint();
    ctrl.ClearPath();

    tp.GenerateLine(QVector3D(400, 300, 300), 2);
    QCOMPARE(tp[0], pose);
}

void TestSimulationController::outOfRangeLatchesUntilReset()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);

    tp.GenerateLine(QVector3D(5000, 5000, 5000), 1); // second point unreachable
    ctrl.Step();                                     // first point: still fine
    QCOMPARE(ctrl.GetState(), SimulationController::Idle);

    ctrl.Step();                                     // fault
    QCOMPARE(ctrl.GetState(), SimulationController::OutOfRange);

    QVector3D pose = kp.GetLastValidPoint();

    // everything except Reset is ignored while faulted
    ctrl.Start();
    QCOMPARE(ctrl.GetState(), SimulationController::OutOfRange);
    ctrl.Step();
    QCOMPARE(kp.GetToolPoint(), pose);
    ctrl.ClearPath();
    QVERIFY(tp.pointsNumber() > 0);
    QCOMPARE(ctrl.AddLinePath(QVector3D(400, 200, 300), 2), false);

    ctrl.Reset();
    QCOMPARE(ctrl.GetState(), SimulationController::Idle);
    QCOMPARE(tp.pointsNumber(), 0);

    // recovered: new paths start from the robot's pose
    QCOMPARE(ctrl.AddLinePath(QVector3D(400, 300, 300), 2), true);
    QCOMPARE(tp[0], pose);
}

// A Settings change can make the current pose unreachable while nothing
// is animating; the fault must latch from Idle too.
void TestSimulationController::faultCanFireFromIdle()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);

    kp.CalculateMachineCoordinates(QVector3D(5000, 5000, 5000));

    QCOMPARE(ctrl.GetState(), SimulationController::OutOfRange);
}

QTEST_GUILESS_MAIN(TestSimulationController)
#include "tst_simulationcontroller.moc"
