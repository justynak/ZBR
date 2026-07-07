#include <QtTest>
#include <QLineEdit>
#include <QPushButton>

#include "widgetanimation.h"
#include "kinematicpoints.h"
#include "trajectorypoints.h"
#include "simulationcontroller.h"

// Widget-level regression tests for the duplicated slot connections:
// setupUi's connectSlotsByName already wires every on_<object>_<signal>
// slot, and the constructor used to connect the same slots again by hand.
// One click fired the handler twice; the static-bool toggles in the Add
// handlers compensated by swallowing every second invocation — so the
// second click did nothing at all. Unit tests can't see connection
// wiring, hence this test drives the real widget (offscreen platform).

class TestWidgetAnimation : public QObject
{
    Q_OBJECT

private slots:
    void everyClickAddsExactlyOnePath();
    void stepButtonAdvancesExactlyOnePoint();
};

void TestWidgetAnimation::everyClickAddsExactlyOnePath()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);
    WidgetAnimation wa;
    wa.SetKinematicPoints(&kp);
    wa.SetTrajectoryPoints(&tp);
    wa.SetController(&ctrl);

    wa.findChild<QLineEdit*>("lineEditStopX")->setText("500");
    wa.findChild<QLineEdit*>("lineEditStopY")->setText("200");
    wa.findChild<QLineEdit*>("lineEditStopZ")->setText("300");
    wa.findChild<QLineEdit*>("lineEditNLine")->setText("4");

    QPushButton *add = wa.findChild<QPushButton*>("buttonLineAdd");
    QVERIFY(add);

    add->click();
    QCOMPARE(tp.pointsNumber(), 5);   // one segment of n+1 points, not two

    add->click();
    QCOMPARE(tp.pointsNumber(), 10);  // the second click must work too
}

// buttonStep had the duplicate connection but no compensating hack:
// one click used to advance the robot two trajectory points.
void TestWidgetAnimation::stepButtonAdvancesExactlyOnePoint()
{
    KinematicPoints kp;
    TrajectoryPoints tp(kp.GetToolPoint());
    SimulationController ctrl(&kp, &tp);
    WidgetAnimation wa;
    wa.SetKinematicPoints(&kp);
    wa.SetTrajectoryPoints(&tp);
    wa.SetController(&ctrl);

    // straight reachable line from the default TCP
    tp.GenerateLine(QVector3D(500, 200, 300), 4);

    QPushButton *step = wa.findChild<QPushButton*>("buttonStep");
    QVERIFY(step);

    step->click();
    step->click();

    QCOMPARE(kp.GetToolPoint(), tp[1]); // two clicks, two points — not four
}

QTEST_MAIN(TestWidgetAnimation)
#include "tst_widgetanimation.moc"
