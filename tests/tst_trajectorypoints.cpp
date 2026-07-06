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
    void lineWithZeroStepsIsRejected();
    void curveSweepsRequestedDegrees();
    void curveFullCircleClosesOnStart();
    void curveStartingAtNegativeXStaysPut();
    void chainedCurveContinuesThroughPole();
    void repeatedDefaultCircleClicksRetrace();
    void chainedDefaultsCloseAndRetrace();
    void lineBreaksTheChain();
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

// n<1 is invalid input: no points appended, no signal emitted
// (previously divided by zero and appended a non-finite point)
void TestTrajectoryPoints::lineWithZeroStepsIsRejected()
{
    TrajectoryPoints tp(QVector3D(0, 0, 0));
    QSignalSpy spy(&tp, SIGNAL(pathGenerated()));

    tp.GenerateLine(QVector3D(10, 0, 0), 0);

    QCOMPARE(tp.pointsNumber(), 0);
    QCOMPARE(spy.count(), 0);
}

// Contract: fi/theta are in degrees, a request of X degrees sweeps
// exactly X degrees of arc. (The old pi/720 factor compensated for
// duplicated slot connections in the UI layer, fixed alongside this.)
void TestTrajectoryPoints::curveSweepsRequestedDegrees()
{
    TrajectoryPoints tp(QVector3D(100, 0, 0));
    QSignalSpy spy(&tp, SIGNAL(pathGenerated()));

    tp.GenerateCurve(QVector3D(0, 0, 0), 90.0, 0.0, 4);

    QCOMPARE(spy.count(), 1);
    QCOMPARE(tp.pointsNumber(), 5);
    QVERIFY(fuzzyEq(tp[0], QVector3D(100, 0, 0)));
    QVERIFY(fuzzyEq(tp[1], QVector3D(92.3880f, 38.2683f, 0)));  // 22.5 deg
    QVERIFY(fuzzyEq(tp[2], QVector3D(70.7107f, 70.7107f, 0)));  // 45 deg
    QVERIFY(fuzzyEq(tp[4], QVector3D(0, 100, 0)));              // 90 deg
}

void TestTrajectoryPoints::curveFullCircleClosesOnStart()
{
    TrajectoryPoints tp(QVector3D(100, 0, 0));

    tp.GenerateCurve(QVector3D(0, 0, 0), 360.0, 0.0, 8);

    QCOMPARE(tp.pointsNumber(), 9);
    QVERIFY(fuzzyEq(tp[8], QVector3D(100, 0, 0)));
    QVERIFY(fuzzyEq(tp[4], QVector3D(-100, 0, 0))); // halfway around
}

// Regression test for the qAtan quadrant bug: an arc starting at x<0
// used to jump to the mirrored azimuth instead of starting at the TCP.
void TestTrajectoryPoints::curveStartingAtNegativeXStaysPut()
{
    TrajectoryPoints tp(QVector3D(-100, 0, 0));

    tp.GenerateCurve(QVector3D(0, 0, 0), 90.0, 0.0, 2);

    QCOMPARE(tp.pointsNumber(), 3);
    QVERIFY(fuzzyEq(tp[0], QVector3D(-100, 0, 0))); // starts at the TCP
    QVERIFY(fuzzyEq(tp[2], QVector3D(0, -100, 0))); // 180deg + 90deg = 270deg
}

// Chained curves must continue from the exact spherical state where the
// previous one ended. Re-deriving angles from the last Cartesian point
// breaks at pole crossings: asin folds the polar angle back into
// [-90,90] and the next arc reverses direction at the seam.
void TestTrajectoryPoints::chainedCurveContinuesThroughPole()
{
    TrajectoryPoints tp(QVector3D(100, 0, 0));

    tp.GenerateCurve(QVector3D(0, 0, 0), 0.0, 90.0, 2); // up to the north pole
    QVERIFY(fuzzyEq(tp[2], QVector3D(0, 0, 100)));

    tp.GenerateCurve(QVector3D(0, 0, 0), 0.0, 90.0, 2); // must keep going

    // continues down the far side instead of bouncing back
    QVERIFY(fuzzyEq(tp[4], QVector3D(-70.7107f, 0, 70.7107f)));
    QVERIFY(fuzzyEq(tp[5], QVector3D(-100, 0, 0)));
}

// The default button (fi=360, theta=0) draws the full circle through the
// TCP; clicking again while the robot is idle retraces the same circle.
void TestTrajectoryPoints::repeatedDefaultCircleClicksRetrace()
{
    TrajectoryPoints tp(QVector3D(400, 200, 300));
    QVector3D center(200, 200, 200);

    tp.GenerateCurve(center, 360.0, 0.0, 36);
    tp.GenerateCurve(center, 360.0, 0.0, 36);

    QCOMPARE(tp.pointsNumber(), 74);
    for(int i = 0; i < 37; ++i)
    {
        // horizontal circle: constant height, constant distance from axis
        QVERIFY(qAbs(tp[i].z() - 300.0f) < 0.01f);
        QVERIFY(qAbs((tp[i] - QVector3D(200, 200, tp[i].z())).length() - 200.0f) < 0.01f);
        // the second click retraces the first
        QVERIFY2(fuzzyEq(tp[i], tp[37 + i], 0.1f),
                 qPrintable(QString("point %1 drifted").arg(i)));
    }
}

// With exact continuation any degree inputs close after finitely many
// clicks: the old UI defaults (fi=40, theta=-120, n=10) close after
// lcm(360/40, 360/120) = 9 clicks and then retrace forever.
void TestTrajectoryPoints::chainedDefaultsCloseAndRetrace()
{
    TrajectoryPoints tp(QVector3D(400, 200, 300));
    QVector3D center(200, 200, 200);

    for(int click = 0; click < 18; ++click)
        tp.GenerateCurve(center, 40.0, -120.0, 10);

    QCOMPARE(tp.pointsNumber(), 18 * 11);
    QVERIFY(fuzzyEq(tp[9 * 11 - 1], QVector3D(400, 200, 300), 0.1f)); // closed
    for(int i = 0; i < 9 * 11; ++i)
        QVERIFY2(fuzzyEq(tp[i], tp[9 * 11 + i], 0.1f),
                 qPrintable(QString("point %1 drifted").arg(i)));
}

// A line segment leaves the sphere, so the next curve must re-derive its
// start from wherever the line ended, not from the stale chain state.
void TestTrajectoryPoints::lineBreaksTheChain()
{
    TrajectoryPoints tp(QVector3D(100, 0, 0));

    tp.GenerateCurve(QVector3D(0, 0, 0), 90.0, 0.0, 4);
    tp.GenerateLine(QVector3D(200, 0, 50), 4);
    int before = tp.pointsNumber();

    tp.GenerateCurve(QVector3D(0, 0, 0), 90.0, 0.0, 4);

    QVERIFY(fuzzyEq(tp[before], QVector3D(200, 0, 50))); // starts at line end
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
