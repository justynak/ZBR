#include <QtTest>
#include <QSignalSpy>
#include <QVector3D>

#include "trajectorypoints.h"

// Characterization tests: they pin current behavior (including quirks)
// as a safety net for refactoring. A failure means behavior changed,
// not necessarily that the new behavior is wrong.

class TestTrajectoryPoints : public QObject
{
    Q_OBJECT

private slots:
    void lineFromCurrentTCP();
    void lineAppendsFromLastPoint();
    void lineWithZeroStepsDividesByZero();
    void curveQuarterFactor();
    void curveZeroRadiusIsNoOp();
    void clearPointsMovesTCPToLastPoint();
    void clearPointsCanKeepTCP();
};

static bool fuzzyEq(const QVector3D &a, const QVector3D &b, float tol = 1e-3f)
{
    return (a - b).length() < tol;
}

void TestTrajectoryPoints::lineFromCurrentTCP()
{
    TrajectoryPoints tp(QVector3D(0, 0, 0));
    QSignalSpy spy(&tp, SIGNAL(pathGenerated()));

    tp.GenerateLine(QVector3D(10, 0, 0), 5);

    QCOMPARE(spy.count(), 1);
    // n steps produce n+1 points, first == currentTCP, last == stop
    QCOMPARE(tp.pointsNumber(), 6);
    QVERIFY(fuzzyEq(tp[0], QVector3D(0, 0, 0)));
    QVERIFY(fuzzyEq(tp[3], QVector3D(6, 0, 0)));
    QVERIFY(fuzzyEq(tp[5], QVector3D(10, 0, 0)));
}

void TestTrajectoryPoints::lineAppendsFromLastPoint()
{
    TrajectoryPoints tp(QVector3D(0, 0, 0));

    tp.GenerateLine(QVector3D(10, 0, 0), 2);
    tp.GenerateLine(QVector3D(10, 10, 0), 2);

    QCOMPARE(tp.pointsNumber(), 6);
    // second segment starts at the previous segment's endpoint,
    // so the shared corner point is duplicated
    QVERIFY(fuzzyEq(tp[2], QVector3D(10, 0, 0)));
    QVERIFY(fuzzyEq(tp[3], QVector3D(10, 0, 0)));
    QVERIFY(fuzzyEq(tp[5], QVector3D(10, 10, 0)));
}

// Known issue, pinned as-is: n=0 divides by zero and appends a single
// non-finite point instead of rejecting the input.
void TestTrajectoryPoints::lineWithZeroStepsDividesByZero()
{
    TrajectoryPoints tp(QVector3D(0, 0, 0));

    tp.GenerateLine(QVector3D(10, 0, 0), 0);

    QCOMPARE(tp.pointsNumber(), 1);
    QVERIFY(qIsNaN(tp[0].x()) || qIsInf(tp[0].x()));
}

// GenerateCurve converts the fi/theta inputs with qAtan(1)/180 = pi/720,
// a quarter of the usual degree-to-radian factor pi/180. Pinned as-is:
// requesting fi=360 over n=4 steps yields a 90-degree arc (22.5 deg/step).
void TestTrajectoryPoints::curveQuarterFactor()
{
    TrajectoryPoints tp(QVector3D(100, 0, 0));
    QSignalSpy spy(&tp, SIGNAL(pathGenerated()));

    tp.GenerateCurve(QVector3D(0, 0, 0), 360.0, 0.0, 4);

    QCOMPARE(spy.count(), 1);
    QCOMPARE(tp.pointsNumber(), 5);
    QVERIFY(fuzzyEq(tp[0], QVector3D(100, 0, 0)));
    QVERIFY(fuzzyEq(tp[1], QVector3D(92.3880f, 38.2683f, 0)));  // 22.5 deg
    QVERIFY(fuzzyEq(tp[2], QVector3D(70.7107f, 70.7107f, 0)));  // 45 deg
    QVERIFY(fuzzyEq(tp[4], QVector3D(0, 100, 0)));              // 90 deg
}

void TestTrajectoryPoints::curveZeroRadiusIsNoOp()
{
    TrajectoryPoints tp(QVector3D(50, 50, 50));
    QSignalSpy spy(&tp, SIGNAL(pathGenerated()));

    tp.GenerateCurve(QVector3D(50, 50, 50), 360.0, 0.0, 4);

    QCOMPARE(tp.pointsNumber(), 0);
    QCOMPARE(spy.count(), 0);
}

void TestTrajectoryPoints::clearPointsMovesTCPToLastPoint()
{
    TrajectoryPoints tp(QVector3D(0, 0, 0));
    tp.GenerateLine(QVector3D(10, 0, 0), 2);

    tp.ClearPoints(true);
    QCOMPARE(tp.pointsNumber(), 0);

    // next path must start from the old endpoint, proving TCP moved
    tp.GenerateLine(QVector3D(10, 10, 0), 1);
    QVERIFY(fuzzyEq(tp[0], QVector3D(10, 0, 0)));
}

void TestTrajectoryPoints::clearPointsCanKeepTCP()
{
    TrajectoryPoints tp(QVector3D(0, 0, 0));
    tp.GenerateLine(QVector3D(10, 0, 0), 2);

    tp.ClearPoints(false);
    QCOMPARE(tp.pointsNumber(), 0);

    tp.GenerateLine(QVector3D(10, 10, 0), 1);
    QVERIFY(fuzzyEq(tp[0], QVector3D(0, 0, 0)));
}

QTEST_GUILESS_MAIN(TestTrajectoryPoints)
#include "tst_trajectorypoints.moc"
