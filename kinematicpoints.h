#ifndef KINEMATICPOINTS_H
#define KINEMATICPOINTS_H

#include <QPoint>
#include <QObject>
#include <QVector3D>
#include <qmath.h>

class KinematicPoints : public QObject
{

public:
    explicit KinematicPoints()
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
        CalculateMachineCoordinates(toolPoint);;
    }



private:
    Q_OBJECT

    // one full inverse-kinematics solution for a candidate tool point,
    // computed by Solve() without touching the committed robot state
    struct IkSolution
    {
        double s[6] = {}, c[6] = {}, fi[6] = {};
        double s23, c23, s234, c234, a, b;
        QVector3D transitionalPoint, regionalPoint;
        bool valid = false;
    };

    IkSolution Solve(QVector3D toolPoint) const;
    void Commit(QVector3D toolPoint, const IkSolution &sol);

    void SetJointPoints();
    void SetCalculatedJointPoints();

    double l[7], d, e;
    double psi, theta;
    double delta1, delta2, delta5;
    double a,b;

    double s[6] = {}, c[6] = {}, ctheta, stheta, spsi, cpsi;
    double fi[6] = {};

    double s23, c23, s234, c234;
    QVector3D joint01Point,joint01prPoint, joint02Point, joint02prPoint, regionalPoint, transitionalPoint, toolPoint;
    QVector3D toolPointCalculated, transitionalPointCalculated, regionalPointCalculated;
    QVector3D lastValidPoint;
    bool handlingOutOfRange = false;

public slots:

    /*SET*/
    void SetL(int l, int n) {this->l[n] = l;}

    void SetD(double dd){this->d = dd;}
    void SetE(double ee){this->e= ee;}

    void SetPsi(double p){this->psi = p; this->cpsi = qCos(psi); this->spsi = qSin(psi);}
    void SetTheta(double t){this->theta = t; this->ctheta = qCos(theta); this->stheta = qSin(theta);}

    void SetDelta1(double d){this->delta1 =d;}
    void SetDelta2(double d){this->delta2 =d;}
    void SetDelta5(double d){this->delta5 =d;}

    void SetToolPoint(QVector3D p) {this->toolPoint = p;}

    void CalculateMachineCoordinates(QVector3D toolPoint);

    // side-effect-free reachability query: no state change, no signals.
    // Runtime failures during a simulation still go through outOfRange.
    bool CanReach(QVector3D p) const { return Solve(p).valid; }

    void Initialize()
    {
        emit statusOK();
    }

    /*GET*/
    //get
    int GetL(int n)
    {
        if(n<0 || n>6) return -1;
        return this->l[n];
    }

    double GetFi(int n)
    {
        if(n<0 || n>5) return -1;
        return this->fi[n];
    }
    double GetS(int n)
    {
        if(n<0 || n>5) return -1;
        return this->s[n];
    }
    double GetC(int n)
    {
        if(n<0 || n>5) return -1;
        return this->c[n];
    }

    double GetD() {return this->d;}
    double GetE() {return this->e;}

    double GetPsi() {return this->psi;}
    double GetTheta() {return this->theta;}

    double GetDelta1() {return this->delta1;}
    double GetDelta2() {return this->delta2;}
    double GetDelta5() {return this->delta5;}

    QList <QVector3D> GetJointPoints()
    {
        QList<QVector3D> list;
        list.append(QVector3D(0,0,0));
        list.append(joint01Point);
        list.append(joint01prPoint);
        list.append(joint02prPoint);
        list.append(joint02Point);
        list.append(regionalPoint);
        list.append(transitionalPoint);
        list.append(toolPoint);

        return list;
    }

    QList <QVector3D> GetCalculatedJointPoints()
    {
        QList<QVector3D> list;
        list.append(regionalPointCalculated);
        list.append(transitionalPointCalculated);
        list.append(toolPointCalculated);
        return list;
    }

    QVector3D GetToolPoint() {return this->toolPoint;}
    QVector3D GetRegionalPoint() {return this->regionalPoint;}
    QVector3D GetTransitionalPoint(){return this->transitionalPoint;}
    QVector3D GetLastValidPoint(){return this->lastValidPoint;}


    //rest

    void RestoreCustomSettings();
signals:
    void statusOK();
    void outOfRange();

};

#endif // KINEMATICPOINTS_H
