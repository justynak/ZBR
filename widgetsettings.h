#ifndef WIDGETSETTINGS_H
#define WIDGETSETTINGS_H

#include <QWidget>
#include "kinematicpoints.h"


namespace Ui {
    class WidgetSettings;
}

class WidgetSettings : public QWidget
{
    Q_OBJECT

public:
    explicit WidgetSettings(QWidget *parent = 0);
    void SetKinematicPoints(KinematicPoints* p );

private:
    Ui::WidgetSettings *ui;
    KinematicPoints* jointPoints;


signals:

public slots:
    void UpdateCalculations();
    void UpdateParameters();
    void OutOfRangeError();

private slots:
    void on_buttonReset_clicked();
};

#endif // WIDGETSETTINGS_H
