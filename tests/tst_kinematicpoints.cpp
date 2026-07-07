#include <QtTest>
#include <QSignalSpy>
#include <QVector3D>

#include "kinematicpoints.h"

// Characterization tests: they pin current behavior (including quirks)
// as a safety net for refactoring. A failure means behavior changed,
// not necessarily that the new behavior is wrong.

class TestKinematicPoints : public QObject
{
    Q_OBJECT

private slots:
    void roundTripReachablePoints_data();
    void roundTripReachablePoints();
    void outOfRangeKeepsLastValidPoint();
    void outOfRangeLeavesRobotAtLastValidPose();
    void outOfRangeEmitsEveryTime();
    void statusOKOnNewValidPoint();
    void statusOKNotEmittedForSamePoint();
    void initializeAlwaysEmitsStatusOK();
    void getLBoundsChecked();
    void canReachIsSideEffectFree();
    void restoreCustomSettingsResetsGeometry();
};

// Inverse kinematics invariant: SetCalculatedJointPoints() runs the forward
// check on the computed joint angles, so for any reachable tool point the
// calculated chain must land back on the input. This survives any internal
// refactoring of the Set* step methods.
void TestKinematicPoints::roundTripReachablePoints_data()
{
    QTest::addColumn<QVector3D>("toolPoint");

    QTest::newRow("default") << QVector3D(400, 200, 300);
    QTest::newRow("shifted-x") << QVector3D(500, 200, 300);
    QTest::newRow("shifted-y") << QVector3D(400, 300, 300);
    QTest::newRow("shifted-z") << QVector3D(400, 200, 400);
    QTest::newRow("low") << QVector3D(450, 250, 150);
    QTest::newRow("high") << QVector3D(350, 150, 500);
}

void TestKinematicPoints::roundTripReachablePoints()
{
    QFETCH(QVector3D, toolPoint);

    KinematicPoints kp;
    QSignalSpy errSpy(&kp, SIGNAL(outOfRange()));

    kp.CalculateMachineCoordinates(toolPoint);

    QVERIFY2(errSpy.count() == 0, "point expected to be reachable");

    QList<QVector3D> calc = kp.GetCalculatedJointPoints();
    QCOMPARE(calc.size(), 3);

    const double tol = 0.5; // mm; QVector3D is float and the chain is long

    QVector3D toolCalc = calc[2];
    QVERIFY2(qAbs(toolCalc.x() - toolPoint.x()) < tol, qPrintable(QString("x: %1 vs %2").arg(toolCalc.x()).arg(toolPoint.x())));
    QVERIFY2(qAbs(toolCalc.y() - toolPoint.y()) < tol, qPrintable(QString("y: %1 vs %2").arg(toolCalc.y()).arg(toolPoint.y())));
    QVERIFY2(qAbs(toolCalc.z() - toolPoint.z()) < tol, qPrintable(QString("z: %1 vs %2").arg(toolCalc.z()).arg(toolPoint.z())));

    // the intermediate frames must agree between IK decomposition and forward check
    QVERIFY((calc[0] - kp.GetRegionalPoint()).length() < tol);
    QVERIFY((calc[1] - kp.GetTransitionalPoint()).length() < tol);
}

void TestKinematicPoints::outOfRangeKeepsLastValidPoint()
{
    KinematicPoints kp;
    QSignalSpy okSpy(&kp, SIGNAL(statusOK()));
    QSignalSpy errSpy(&kp, SIGNAL(outOfRange()));

    QVector3D valid = kp.GetLastValidPoint();
    kp.CalculateMachineCoordinates(QVector3D(5000, 5000, 5000));

    QCOMPARE(errSpy.count(), 1);
    QCOMPARE(okSpy.count(), 0);
    QCOMPARE(kp.GetLastValidPoint(), valid);
}

// Design decision: the robot stops BEFORE moving to an unreachable point.
// A failed candidate must never remain in the object's state — after a
// rejected point every getter still reflects the pose at lastValidPoint.
void TestKinematicPoints::outOfRangeLeavesRobotAtLastValidPose()
{
    KinematicPoints kp;
    QVector3D valid(420, 210, 310);
    kp.CalculateMachineCoordinates(valid);
    QList<QVector3D> pose = kp.GetJointPoints();

    QSignalSpy errSpy(&kp, SIGNAL(outOfRange()));
    kp.CalculateMachineCoordinates(QVector3D(5000, 5000, 5000));

    QCOMPARE(errSpy.count(), 1);
    QCOMPARE(kp.GetToolPoint(), valid);

    QList<QVector3D> after = kp.GetJointPoints();
    QCOMPARE(after.size(), pose.size());
    for(int i = 0; i < pose.size(); ++i)
        QVERIFY2((after[i] - pose[i]).length() < 0.5,
                 qPrintable(QString("joint %1 moved").arg(i)));
}

// Regression test for the old `static bool ok` in CalculateMachineCoordinates:
// it swallowed every second failure, so consecutive out-of-range points
// alternated between "signal" and "silence".
void TestKinematicPoints::outOfRangeEmitsEveryTime()
{
    KinematicPoints kp;
    QSignalSpy errSpy(&kp, SIGNAL(outOfRange()));

    kp.CalculateMachineCoordinates(QVector3D(5000, 5000, 5000));
    kp.CalculateMachineCoordinates(QVector3D(-4000, 4000, 4000));
    kp.CalculateMachineCoordinates(QVector3D(0, 0, 9000));

    QCOMPARE(errSpy.count(), 3);
}

void TestKinematicPoints::statusOKOnNewValidPoint()
{
    KinematicPoints kp;
    QSignalSpy okSpy(&kp, SIGNAL(statusOK()));

    QVector3D p(420, 210, 310);
    kp.CalculateMachineCoordinates(p);

    QCOMPARE(okSpy.count(), 1);
    QCOMPARE(kp.GetLastValidPoint(), p);
}

// Pins the `lastValidPoint != toolPoint` guard: recalculating the same point
// (e.g. after a settings change) does NOT re-emit statusOK. This is also why
// the constructor's CalculateMachineCoordinates call emits nothing.
void TestKinematicPoints::statusOKNotEmittedForSamePoint()
{
    KinematicPoints kp;
    QSignalSpy okSpy(&kp, SIGNAL(statusOK()));

    QVector3D p(420, 210, 310);
    kp.CalculateMachineCoordinates(p);
    kp.CalculateMachineCoordinates(p);

    QCOMPARE(okSpy.count(), 1);
}

// Initialize() exists to trigger the first paint after signals are connected
// (MainWindow: construct -> connect -> Initialize). Load-bearing for startup.
void TestKinematicPoints::initializeAlwaysEmitsStatusOK()
{
    KinematicPoints kp;
    QSignalSpy okSpy(&kp, SIGNAL(statusOK()));

    kp.Initialize();

    QCOMPARE(okSpy.count(), 1);
}

// CanReach is a query, not a command: no signals fire during the check
// and the committed state — including lastValidPoint — stays untouched.
// (The naive implementation, "run CalculateMachineCoordinates and restore
// afterwards", would latch a fault in every connected observer and
// overwrite lastValidPoint.)
void TestKinematicPoints::canReachIsSideEffectFree()
{
    KinematicPoints kp;
    QVector3D pose(420, 210, 310);
    kp.CalculateMachineCoordinates(pose);

    QSignalSpy okSpy(&kp, SIGNAL(statusOK()));
    QSignalSpy errSpy(&kp, SIGNAL(outOfRange()));
    QList<QVector3D> joints = kp.GetJointPoints();

    QCOMPARE(kp.CanReach(QVector3D(5000, 5000, 5000)), false);
    QCOMPARE(kp.CanReach(QVector3D(350, 150, 250)), true);

    QCOMPARE(okSpy.count(), 0);
    QCOMPARE(errSpy.count(), 0);
    QCOMPARE(kp.GetLastValidPoint(), pose);
    QCOMPARE(kp.GetToolPoint(), pose);
    QCOMPARE(kp.GetJointPoints(), joints);
}

void TestKinematicPoints::getLBoundsChecked()
{
    KinematicPoints kp;

    QCOMPARE(kp.GetL(-1), -1);
    QCOMPARE(kp.GetL(7), -1);
    QCOMPARE(kp.GetL(1), 250);
    QCOMPARE(kp.GetL(3), 550);
    QCOMPARE(kp.GetL(6), 80);

    QCOMPARE(kp.GetFi(-1), -1.0);
    QCOMPARE(kp.GetS(6), -1.0);
    QCOMPARE(kp.GetC(-1), -1.0);
}

void TestKinematicPoints::restoreCustomSettingsResetsGeometry()
{
    KinematicPoints kp;
    kp.SetL(999, 3);
    kp.SetE(123.0);

    kp.RestoreCustomSettings();

    QCOMPARE(kp.GetL(3), 550);
    QCOMPARE(kp.GetE(), 200.0);
    QCOMPARE(kp.GetToolPoint(), QVector3D(400, 200, 300));
    QCOMPARE(kp.GetLastValidPoint(), QVector3D(400, 200, 300));
}

QTEST_GUILESS_MAIN(TestKinematicPoints)
#include "tst_kinematicpoints.moc"
