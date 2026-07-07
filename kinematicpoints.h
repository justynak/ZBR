#ifndef KINEMATICPOINTS_H
#define KINEMATICPOINTS_H

#include <QPoint>
#include <QObject>
#include <QVector3D>
#include <qmath.h>

class KinematicPoints : public QObject
{

public:
    // the robot's dimensions as a value object, so a candidate geometry
    // can be solved against without committing it (Settings transaction)
    struct Geometry
    {
        double l[7];
        double d, e;
        double psi, theta;
        double delta1, delta2, delta5;
    };

    static Geometry DefaultGeometry()
    {
        Geometry g;
        g.l[0] = 0.0;
        g.l[1] = 250.0;
        g.l[2] = 250.0;
        g.l[3] = 550.0;
        g.l[4] = 100.0;
        g.l[5] = 100.0;
        g.l[6] = 80.0;

        g.d = 50.0;
        g.e = 200.0;

        //radians
        g.psi = 10*3.14/180;
        g.theta = 30*3.14/180;

        g.delta1 = -1.0;
        g.delta2 = -1.0;
        g.delta5 = -1.0;
        return g;
    }

    explicit KinematicPoints()
    {
        geo = DefaultGeometry();

        toolPoint = QVector3D(400, 200, 300);
        lastValidPoint = toolPoint;
        CalculateMachineCoordinates(toolPoint);
    }

    Geometry GetGeometry() const { return geo; }

    // Settings transaction: commit the new geometry only if the robot's
    // current tool point stays reachable under it. On rejection nothing
    // changes and no fault is raised — the robot never entered an invalid
    // configuration. On success emits geometryChanged (so the path can be
    // revalidated) and statusOK (so the views repaint the new pose).
    bool TrySetGeometry(const Geometry &g);

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

    IkSolution Solve(QVector3D toolPoint, const Geometry &g) const;
    void Commit(QVector3D toolPoint, const IkSolution &sol);

    void SetJointPoints();
    void SetCalculatedJointPoints();

    Geometry geo;
    double a,b;

    double s[6] = {}, c[6] = {};
    double fi[6] = {};

    double s23, c23, s234, c234;
    QVector3D joint01Point,joint01prPoint, joint02Point, joint02prPoint, regionalPoint, transitionalPoint, toolPoint;
    QVector3D toolPointCalculated, transitionalPointCalculated, regionalPointCalculated;
    QVector3D lastValidPoint;
    bool handlingOutOfRange = false;

public slots:

    /*SET*/
    // direct setters bypass the Settings transaction; the UI goes through
    // TrySetGeometry instead
    void SetL(int l, int n) {this->geo.l[n] = l;}

    void SetD(double dd){this->geo.d = dd;}
    void SetE(double ee){this->geo.e = ee;}

    void SetPsi(double p){this->geo.psi = p;}
    void SetTheta(double t){this->geo.theta = t;}

    void SetDelta1(double d){this->geo.delta1 = d;}
    void SetDelta2(double d){this->geo.delta2 = d;}
    void SetDelta5(double d){this->geo.delta5 = d;}

    void SetToolPoint(QVector3D p) {this->toolPoint = p;}

    void CalculateMachineCoordinates(QVector3D toolPoint);

    // side-effect-free reachability query: no state change, no signals.
    // Runtime failures during a simulation still go through outOfRange.
    bool CanReach(QVector3D p) const { return Solve(p, geo).valid; }

    void Initialize()
    {
        emit statusOK();
    }

    /*GET*/
    //get
    int GetL(int n)
    {
        if(n<0 || n>6) return -1;
        return this->geo.l[n];
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

    double GetD() {return this->geo.d;}
    double GetE() {return this->geo.e;}

    double GetPsi() {return this->geo.psi;}
    double GetTheta() {return this->geo.theta;}

    double GetDelta1() {return this->geo.delta1;}
    double GetDelta2() {return this->geo.delta2;}
    double GetDelta5() {return this->geo.delta5;}

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
    void geometryChanged();

};

#endif // KINEMATICPOINTS_H
