#ifndef TRAJECTORYPOINTS_H
#define TRAJECTORYPOINTS_H


#include <QList>
#include <QVector3D>
#include <QObject>

class TrajectoryPoints : public QObject
{

public:
    TrajectoryPoints(QVector3D startPoint)
    {
        currentTCP = startPoint;
    }

    QVector3D& operator[] (int n)
    {
        return points[n];
    }

    int pointsNumber() { return points.size();}

    QList <QVector3D> GetPointList(){return points;}

    Q_OBJECT
private:
    QList <QVector3D> points;
    QVector3D currentTCP;

    // exact spherical state where the last curve ended; asin/atan2 cannot
    // reconstruct it from a Cartesian point once a sweep crosses a pole,
    // so chained curves continue from here instead of re-deriving
    bool chainValid = false;
    QVector3D chainCenter;
    double chainRadius = 0.0;
    double chainTheta = 0.0;
    double chainFi = 0.0;

signals:
    void pathGenerated();

public slots:
    void GenerateCurve(QVector3D center, double fi, double theta, int n);
    void GenerateLine(QVector3D stop, int n);
    void SetNewTCP(QVector3D newPoint) {currentTCP = newPoint; chainValid = false;}
    void ClearPoints(bool changeCurrentTCP){
        chainValid = false;
        if(! points.isEmpty())
        {
            if(changeCurrentTCP) currentTCP = points.takeLast();
            points.clear();
        }
    }
    void RemovePoint() {points.removeFirst();}

    // drop everything from index count on (used when the tail of a
    // generated path is unreachable); the spherical chain state no longer
    // matches the new end, so the next curve re-derives from Cartesian
    void Truncate(int count)
    {
        while(points.size() > count)
            points.removeLast();
        chainValid = false;
    }

    void Append(QVector3D p) { points.append(p); }


};

#endif // TRAJECTORYPOINTS_H
