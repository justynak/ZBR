#include "kinematicpoints.h"

#include <qmath.h>

#include <QDebug>


// Pure inverse kinematics: computes the full solution for a candidate
// tool point from the current geometry, without touching the committed
// robot state and without emitting signals. An unreachable point shows
// up as NaN joint angles (negative values under the square roots).
KinematicPoints::IkSolution KinematicPoints::Solve(QVector3D tool, const Geometry &g) const
{
    IkSolution r;

    const double *l = g.l;
    double e = g.e, delta1 = g.delta1, delta2 = g.delta2, delta5 = g.delta5;
    double cpsi = qCos(g.psi), spsi = qSin(g.psi);
    double ctheta = qCos(g.theta), stheta = qSin(g.theta);

    // transitional point: tool minus the tool/wrist segment
    double px = tool.x() - (l[5] + l[6]) * ctheta * cpsi;
    double py = tool.y() - (l[5] + l[6]) * ctheta * spsi;
    double pz = tool.z() - (l[5] + l[6]) * stheta;
    r.transitionalPoint = QVector3D(px, py, pz);

    r.s[1] = 1/(qPow(px,2) + qPow(py,2)) * (e*px + delta1 * py * qSqrt(qPow(px,2) + qPow(py,2) - qPow(e,2)));
    r.c[1] = 1/(qPow(px,2) + qPow(py,2)) * (e*py*(-1.0) + delta1 * px * qSqrt(qPow(px,2) + qPow(py,2) - qPow(e,2)));

    r.s[5] = ctheta * (spsi * r.c[1] - cpsi * r.s[1]);
    r.c[5] = delta5 * qSqrt(1 - qPow(r.s[5], 2));

    r.s234 = stheta / r.c[5];
    r.c234 = ctheta / r.c[5] * (cpsi * r.c[1] + spsi * r.s[1]);

    r.regionalPoint = QVector3D(px - l[4] * r.c[1] * r.c234,
                                py - l[4] * r.s[1] * r.c234,
                                pz - l[4] * r.s234);

    double xr = r.regionalPoint.x();
    double yr = r.regionalPoint.y();
    double zr = r.regionalPoint.z();

    r.a = (-1.0)*l[1] + delta1 * qSqrt(qPow(xr, 2) + qPow(yr, 2) - qPow(e, 2));
    r.b = 0.5/l[2] * (qPow(r.a, 2) + qPow(zr, 2) + qPow(l[2], 2) - qPow(l[3], 2));

    r.s[2] = 1/(qPow(r.a, 2) + qPow(zr,2)) * (zr * r.b + delta2 * r.a * qSqrt(qPow(r.a, 2) + qPow(zr,2) - qPow(r.b,2)));
    r.c[2] = 1/(qPow(r.a, 2) + qPow(zr,2)) * (r.a * r.b - delta2 * zr * qSqrt(qPow(r.a, 2) + qPow(zr,2) - qPow(r.b,2)));

    r.s[3] = 1/l[3] * (zr*r.c[2] - r.a*r.s[2]);
    r.c[3] = 1/l[3] * (r.a*r.c[2] + zr*r.s[2] - l[2]);

    r.s23 = 1/l[3] * (zr - l[2]*r.s[2]);
    r.c23 = 1/l[3] * (r.a - l[2]*r.c[2]);

    r.s[4] = r.s234 * r.c23 - r.c234 * r.s23;
    r.c[4] = r.c234 * r.c23 + r.s234 * r.s23;

    // joint angles are fi[1..5]; index 0 is unused
    r.valid = true;
    for(int i=1; i<=5; ++i)
    {
        r.fi[i] = r.c[i] > r.s[i] ? qAsin(r.s[i]) : qAcos(r.c[i]);
        if(r.fi[i] != r.fi[i]) r.valid = false;
    }

    return r;
}

void KinematicPoints::Commit(QVector3D tool, const IkSolution &sol)
{
    toolPoint = tool;
    transitionalPoint = sol.transitionalPoint;
    regionalPoint = sol.regionalPoint;

    for(int i=0; i<6; ++i)
    {
        s[i] = sol.s[i];
        c[i] = sol.c[i];
        fi[i] = sol.fi[i];
    }
    s23 = sol.s23;
    c23 = sol.c23;
    s234 = sol.s234;
    c234 = sol.c234;
    a = sol.a;
    b = sol.b;

    SetJointPoints();
    SetCalculatedJointPoints();
}

//calculate points
void KinematicPoints::SetJointPoints()
{
    const double *l = geo.l;
    double d = geo.d, e = geo.e;

    joint01Point = QVector3D(l[1]*c[1], l[1]*s[1], 0);
    joint01prPoint = QVector3D(joint01Point.x() + d*s[1], joint01Point.y() - d*c[1], 0);
    joint02prPoint = QVector3D(joint01prPoint.x()+l[2]*c[2]*c[1], joint01prPoint.y()+l[2]*c[2]*s[1], l[2]*s[2]);
    joint02Point = QVector3D(joint02prPoint.x() - (d-e)*s[1], joint02prPoint.y() + (d-e)*c[1], joint02prPoint.z());
}

//check points
void KinematicPoints::SetCalculatedJointPoints()
{
    const double *l = geo.l;
    double cpsi = qCos(geo.psi), spsi = qSin(geo.psi);
    double ctheta = qCos(geo.theta), stheta = qSin(geo.theta);

    regionalPointCalculated = QVector3D(joint02Point.x() + l[3]*c[1]*c23, joint02Point.y()+l[3]*s[1]*c23, joint02Point.z()+l[3]*s23);
    transitionalPointCalculated = QVector3D(regionalPointCalculated.x() + l[4]*c[1]*c234, regionalPointCalculated.y() + l[4]*s[1]*c234, regionalPointCalculated.z()+l[4]*s234);
    toolPointCalculated = QVector3D(transitionalPointCalculated.x() + (l[5]+l[6])*ctheta*cpsi, transitionalPointCalculated.y() + (l[5]+l[6])*ctheta*spsi, transitionalPointCalculated.z()+(l[5]+l[6])*stheta);
}


void KinematicPoints::CalculateMachineCoordinates(QVector3D toolPoint)
{
    IkSolution sol = Solve(toolPoint, geo);

    // validate, then commit: a rejected candidate never touches the
    // robot's state, so the pose always stays at lastValidPoint
    if(!sol.valid)
    {
        // the guard stops infinite recursion if an outOfRange handler
        // re-enters with another unreachable point
        if(!handlingOutOfRange)
        {
            handlingOutOfRange = true;
            emit outOfRange();
            handlingOutOfRange = false;
        }
        return;
    }

    Commit(toolPoint, sol);

    if(lastValidPoint != toolPoint)
        emit statusOK();

    lastValidPoint = toolPoint;
}

void KinematicPoints::RestoreCustomSettings()
{
    geo = DefaultGeometry();

    toolPoint = QVector3D(400, 200, 300);
    lastValidPoint = toolPoint;
    CalculateMachineCoordinates(toolPoint);
    emit geometryChanged();
}

bool KinematicPoints::TrySetGeometry(const Geometry &g)
{
    IkSolution sol = Solve(toolPoint, g);
    if(!sol.valid)
        return false; // nothing committed, no fault: the change never happened

    geo = g;
    Commit(toolPoint, sol);
    emit geometryChanged();
    emit statusOK(); // same pose, new joint positions: force a repaint
    return true;
}
