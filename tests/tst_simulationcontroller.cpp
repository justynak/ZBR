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
    void startGivesImmediateFeedback();
    void defaultCircleRunsToCompletion();
    void clearWhileRunningStopsEverything();
    void newPathAfterClearStartsFromRobotPose();
    void outOfRangeLatchesUntilReset();
    void faultCanFireFromIdle();
    void unreachablePathIsTrimmedAtAddTime();
    void reachablePathIsNotTrimmed();
    void trimmedPathRunsWithoutFault();
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
    ctrl.Start();            // advances tp[0] synchronously
    QCOMPARE(ctrl.GetState(), SimulationController::Running);

    ctrl.Step();

    QCOMPARE(ctrl.GetState(), SimulationController::Idle);
    QCOMPARE(kp.GetToolPoint(), tp[1]);
}

// Start must show life on the click itself: the first point is advanced
// synchronously, so the progress bar moves before the first interval —
// and a path that faults immediately turns the label red immediately,
// instead of looking like a dead Start button.
void TestSimulationController::startGivesImmediateFeedback()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);
    QSignalSpy progress(&ctrl, SIGNAL(progressChanged(int)));

    tp.GenerateLine(QVector3D(500, 200, 300), 4);
    ctrl.SetInterval(60000);
    ctrl.Start();

    QCOMPARE(progress.count(), 1); // emitted on the click, not after 60 s
}

// Out-of-box experience: the circle-button defaults (center 350,200,300,
// fi=360, theta=0, n=36) must lie entirely inside the default workspace.
// The previous default center (200,200,200) faulted on 12 of 37 points.
void TestSimulationController::defaultCircleRunsToCompletion()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);
    QSignalSpy states(&ctrl, SIGNAL(stateChanged(int)));

    ctrl.AddCurvePath(QVector3D(350, 200, 300), 360.0, 0.0, 36);
    ctrl.SetInterval(0);
    ctrl.Start();

    QTRY_COMPARE(ctrl.GetState(), SimulationController::Idle);

    for(int i = 0; i < states.count(); ++i)
        QVERIFY(states.at(i).first().toInt() != SimulationController::OutOfRange);
    // full circle: the robot is back where it started
    QVERIFY((kp.GetToolPoint() - QVector3D(400, 200, 300)).length() < 0.1f);
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

// Paths are validated when added, not discovered bad mid-drive: the
// unreachable tail is cut at the last reachable point and pathTrimmed
// carries that point for the UI. Not a fault — the robot never moved.
void TestSimulationController::unreachablePathIsTrimmedAtAddTime()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);
    QSignalSpy trimmed(&ctrl, SIGNAL(pathTrimmed(QVector3D)));

    QVector3D start = kp.GetToolPoint();
    QCOMPARE(ctrl.AddLinePath(QVector3D(5000, 5000, 5000), 10), true);

    QCOMPARE(trimmed.count(), 1);
    QVERIFY(tp.pointsNumber() >= 2); // seed + refined boundary point
    QVERIFY(tp.pointsNumber() < 11);
    QCOMPARE(ctrl.GetState(), SimulationController::Idle); // not a fault

    // the signal carries the new path end, and every survivor is reachable
    QVector3D end = tp[tp.pointsNumber()-1];
    QCOMPARE(trimmed.first().first().value<QVector3D>(), end);
    for(int i = 0; i < tp.pointsNumber(); ++i)
        QVERIFY(kp.CanReach(tp[i]));

    // the trim is refined to the workspace boundary, not to the last
    // coarse sample (which here would be the robot's own position):
    // the robot gets real travel, and a step further is out of range
    QVERIFY((end - start).length() > 10.0f);
    QVector3D dir = (QVector3D(5000, 5000, 5000) - start).normalized();
    QVERIFY(!kp.CanReach(end + dir * 5.0f));

    // and the robot actually walks to the boundary
    ctrl.SetInterval(0);
    ctrl.Start();
    QTRY_COMPARE(ctrl.GetState(), SimulationController::Idle);
    QCOMPARE(kp.GetToolPoint(), end);
}

void TestSimulationController::reachablePathIsNotTrimmed()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);
    QSignalSpy trimmed(&ctrl, SIGNAL(pathTrimmed(QVector3D)));

    // radius-70 circle from the line's end at (500,200,300): stays well
    // clear of the base column exclusion zone
    ctrl.AddLinePath(QVector3D(500, 200, 300), 4);
    ctrl.AddCurvePath(QVector3D(450, 250, 300), 360.0, 0.0, 36);

    QCOMPARE(trimmed.count(), 0);
}

// The payoff: the old default circle always faulted mid-run; the same
// input is now trimmed at add time and what remains runs to completion.
void TestSimulationController::trimmedPathRunsWithoutFault()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);
    QSignalSpy states(&ctrl, SIGNAL(stateChanged(int)));

    ctrl.AddCurvePath(QVector3D(200, 200, 200), 360.0, 0.0, 36); // 12/37 unreachable
    QVERIFY(tp.pointsNumber() < 37);

    ctrl.SetInterval(0);
    ctrl.Start();
    QTRY_COMPARE(ctrl.GetState(), SimulationController::Idle);

    for(int i = 0; i < states.count(); ++i)
        QVERIFY(states.at(i).first().toInt() != SimulationController::OutOfRange);
}

QTEST_GUILESS_MAIN(TestSimulationController)
#include "tst_simulationcontroller.moc"
