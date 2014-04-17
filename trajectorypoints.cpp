#include "trajectorypoints.h"

#include <qmath.h>

void TrajectoryPoints::GenerateCurve(QVector3D center, double fi, double theta, int n)
{

    double stepFi =fi/n*qAtan(1)/180.0;
    double stepTheta = theta/n*qAtan(1)/180.0;

    QVector3D referencePoint;

    if(points.isEmpty())
        referencePoint = currentTCP;
    else
        referencePoint = points.last();

    QVector3D pointInSphere = QVector3D(referencePoint.x() - center.x(),referencePoint.y() - center.y(),referencePoint.z() - center.z());

    double radius = qSqrt(qPow(pointInSphere.x(),2) + qPow(pointInSphere.y(),2) + qPow(pointInSphere.z(),2));
    if(radius == 0.0) return;

    double startTheta = qAsin(( pointInSphere.z()/radius ));

    double startFi=0.0;
    if (pointInSphere.x()!=0)
    {
        startFi = qAtan((pointInSphere.y()/(pointInSphere.x())));
    }

    //points.append(currentTCP);

    for(int i=0; i<n+1; ++i)
    {
        points.append(center +
                      radius * QVector3D(
                          qCos(startTheta + stepTheta * i) * qCos(startFi + stepFi * i),
                          qCos(startTheta + stepTheta * i) * qSin(startFi + stepFi * i),
                          qSin(startTheta + stepTheta * i)
                      )
                          );
    }

    emit pathGenerated();
}

void TrajectoryPoints::GenerateLine(QVector3D stop, int n)
{
    QVector3D reference;
    if(points.isEmpty())
        reference = currentTCP;
    else
        reference = points.last();

    QVector3D step = (-reference+stop)/n;

    for(int i=0; i<n+1; ++i)
    {
        points.append(reference + step*i);
    }

    emit pathGenerated();
}

