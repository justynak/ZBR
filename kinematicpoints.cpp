#include "kinematicpoints.h"

#include <qmath.h>

#include <QDebug>



/*CALCULATIONS*/
void KinematicPoints::SetRegionalPoint()
{
    double rx = transitionalPoint.x() - l[4] * c[1] * c234;
    double ry = transitionalPoint.y() - l[4] * s[1] * c234;
    double rz = transitionalPoint.z() - l[4] * s234;

    this->regionalPoint = QVector3D(rx, ry, rz);
}

void KinematicPoints::SetTransitionalPoint()
{
    double px = toolPoint.x() - (l[5] + l[6]) * ctheta * cpsi ;
    double py = toolPoint.y() - (l[5] + l[6]) * ctheta * spsi ;
    double pz = toolPoint.z() - (l[5] + l[6]) * stheta ;

    this->transitionalPoint = QVector3D(px, py, pz);
}


//angles
void KinematicPoints::SetS1C1()
{
    double px = this->transitionalPoint.x();
    double py = this->transitionalPoint.y();

    s[1] = 1/( qPow(px,2) + qPow(py,2) ) * (e*px + delta1 * py * qSqrt(qPow(px,2) + qPow(py,2) - qPow(e,2)) );
    c[1] = 1/( qPow(px,2) + qPow(py,2) ) * (e*py*(-1.0) + delta1 * px * (qSqrt(qPow(px,2) + qPow(py,2) - qPow(e,2))));

    //if(fi[1]!=fi[1]) {emit outOfRange(); return;}
}

void KinematicPoints::SetS2C2()
{
    double zr = regionalPoint.z();

    s[2] = 1/(qPow(a, 2) + qPow(zr,2)) * (zr * b + delta2 * a * qSqrt(qPow(a, 2) + qPow(zr,2) - qPow(b,2)));
    c[2] = 1/(qPow(a, 2) + qPow(zr,2)) * (a * b + delta2 * zr * qSqrt(qPow(a, 2) + qPow(zr,2) - qPow(b,2)) );
    fi[2] = qFabs(c[2])>qFabs(s[2])? qAsin(s[2]) : qAcos(c[2]);

    //if(fi[2]!=fi[2]) {emit outOfRange(); return;}

}

void KinematicPoints::SetS3C3()
{
    double zr = regionalPoint.z();

   s[3] = 1/l[3] * ( zr*c[2] - a*s[2] );
   c[3] = 1/l[3]* (a*c[2]+zr*s[2] - l[2]) ;
   fi[3] = qFabs(c[3])>qFabs(s[3])? qAsin(s[3]) : qAcos(c[3]);

   //if(fi[3]!=fi[3]) {emit outOfRange(); return;}

}

void KinematicPoints::SetS4C4()
{
    s[4] = s234 * c23 - c234 * s23;
    c[4] = c234*c23 - s234*s23;
    fi[4] = qFabs(c[4])>qFabs(s[4])? qAsin(s[4]) : qAcos(c[4]);
    //if(fi[4]!=fi[4]) {emit outOfRange(); return;}

}

void KinematicPoints::SetS5C5()
{
    s[5] = ctheta * (spsi * c[1] - cpsi * s[1]);
    c[5] = delta5 * qSqrt(1 - qPow(s[5], 2));
    fi[5] = qFabs(c[5])>qFabs(s[5])? qAsin(s[5]) : qAcos(c[5]);
    //if(fi[5]!=fi[5]) {emit outOfRange(); return;}

}

void KinematicPoints::SetS23C23()
{
    double zr = regionalPoint.z();
    s23 = 1/l[3] *(zr - l[2]*s[2]);
    c23 = 1/l[3] * (a - l[2]*c[2]);

}

void KinematicPoints::SetS234C234()
{

        s234= stheta / c[5];
        c234= ctheta / c[5]* (cpsi*c[1] +spsi*s[1]);
}

void KinematicPoints::SetAngles()
{

}

void KinematicPoints::SetAB()
{
    double xr = regionalPoint.x();
    double yr = regionalPoint.y();
    double zr = regionalPoint.z();

    a = (-1.0)*l[1] + delta1* qSqrt(qPow(xr, 2) + qPow(yr, 2) - qPow(e, 2));
    b = 0.5/l[2] * (qPow(a, 2) + qPow(zr, 2) + qPow(l[2], 2) - qPow(l[3], 2));
}

//calculate points
void KinematicPoints::SetJointPoints()
{
    joint01Point = QVector3D(l[1]*c[1], l[1]*s[1], 0);
    joint01prPoint = QVector3D(joint01Point.x() + d*s[1], joint01Point.y() - d*c[1], 0);
    joint02prPoint = QVector3D(joint01prPoint.x()+l[2]*c[2]*c[1], joint01prPoint.y()+l[2]*c[2]*s[1], l[2]*s[2]);
    joint02Point = QVector3D(joint02prPoint.x() - (d-e)*s[1], joint02prPoint.y() + (d-e)*c[1], joint02prPoint.z());
}

//check points
void KinematicPoints::SetCalculatedJointPoints()
{
    regionalPointCalculated = QVector3D(joint02Point.x() + l[3]*c[1]*c23, joint02Point.y()+l[3]*s[1]*c23, joint02Point.z()+l[3]*s23);
    transitionalPointCalculated = QVector3D(regionalPointCalculated.x() + l[4]*c[1]*c234, regionalPointCalculated.y() + l[4]*s[1]*c234, regionalPointCalculated.z()+l[4]*s234);
    toolPointCalculated = QVector3D(transitionalPointCalculated.x() + (l[5]+l[6])*ctheta*cpsi, transitionalPointCalculated.y() + (l[5]+l[6])*ctheta*spsi, transitionalPointCalculated.z()+(l[5]+l[6])*stheta);
}


void KinematicPoints::CalculateMachineCoordinates(QVector3D toolPoint)
{
    SetToolPoint(toolPoint);
    SetTransitionalPoint();
    SetS1C1();
    SetS5C5();
    SetS234C234();
    SetRegionalPoint();
    SetAB();
    SetS2C2();
    SetS3C3();
    SetS23C23();
    SetS4C4();
    SetJointPoints();
    SetCalculatedJointPoints();

   static bool ok = true;
   for(int i=0; i<5; ++i)
   {
      fi[i] = c[i]>s[i]? qAsin(s[i]) : qAcos(c[i]);
      if(fi[i]!=fi[i])
      {
         if(ok)
         {
             ok = false;
             emit outOfRange();
             return;
         }

        else
         {
             ok = true;
             return;
         }
        }
   }

    if(lastValidPoint != toolPoint)
        emit statusOK();

    lastValidPoint = toolPoint;

}

void KinematicPoints::RestoreCustomSettings()
{
    l[1] = 250.0;
    l[2] = 250.0;
    l[3] = 550.0;
    l[4] = 100.0;
    l[5] = 100.0;
    l[6] = 80.0;

    d = 50.0;
    e = 200.0;

    delta1 = -1.0;
    delta2 = -1.0;
    delta5 = -1.0;

    //radians
    SetPsi(10*3.14/180);
    SetTheta(30*3.14/180);

    toolPoint = QVector3D(400, 200, 300);
    lastValidPoint = toolPoint;
    CalculateMachineCoordinates(toolPoint);
}
