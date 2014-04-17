#include "widgetsettings.h"
#include "ui_widgetsettings.h"


#include <QMessageBox>
/*
WidgetAnimation::WidgetAnimation(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::WidgetAnimation)
  */

WidgetSettings::WidgetSettings(QWidget *parent) :
    QWidget(parent), ui(new Ui::WidgetSettings)
{
    ui->setupUi(this);
    QPixmap pix = QPixmap(70,50);
    pix.fill(Qt::darkGreen);
    ui->labelValid->setPixmap(pix);
    ui->labelRobotPic->setPixmap(QPixmap("robot.png"));
}

void WidgetSettings::SetKinematicPoints(KinematicPoints *p)
{
    jointPoints = p;
    connect(jointPoints, SIGNAL(statusOK()), this, SLOT(UpdateCalculations()));
    connect(jointPoints, SIGNAL(outOfRange()), this, SLOT(OutOfRangeError()));
    connect(ui->buttonSet, SIGNAL(clicked()), this, SLOT(UpdateParameters()));
    connect(ui->buttonReset, SIGNAL(clicked()), this, SLOT(on_buttonReset_clicked()));

}

void WidgetSettings::UpdateCalculations()
{

    ui->listWidget->clear();
    ui->listWidget->addItem(tr("fi1 = \t%1").arg(jointPoints->GetFi(1)));
    ui->listWidget->addItem(tr("fi2 = \t%1").arg(jointPoints->GetFi(2)));
    ui->listWidget->addItem(tr("fi3 = \t%1").arg(jointPoints->GetFi(3)));
    ui->listWidget->addItem(tr("fi4 = \t%1").arg(jointPoints->GetFi(4)));
    ui->listWidget->addItem(tr("fi5 = \t%1").arg(jointPoints->GetFi(5)));
    ui->listWidget->addItem(tr(""));

    ui->listWidget->addItem(tr("S1, C1 =\t%1, \t%2").arg(jointPoints->GetS(1)).arg(jointPoints->GetC(1)));
    ui->listWidget->addItem(tr("S2, C2 =\t %1,\t%2").arg(jointPoints->GetS(2)).arg(jointPoints->GetC(2)));
    ui->listWidget->addItem(tr("S3, C3 =\t%1, \t%2").arg(jointPoints->GetS(3)).arg(jointPoints->GetC(3)));
    ui->listWidget->addItem(tr("S4, C4 =\t%1,\t%2").arg(jointPoints->GetS(4)).arg(jointPoints->GetC(4)));
    ui->listWidget->addItem(tr("S5, C5 =\t%1,\t%2").arg(jointPoints->GetS(5)).arg(jointPoints->GetC(5)));

    ui->listWidget->addItem(tr(""));

    //joint points
    QList <QVector3D> list = jointPoints->GetJointPoints();
    ui->listWidget->addItem(tr("x01=\t (%1, %2, %3)").arg(list[1].x()).arg(list[1].y()).arg(list[1].z()));
    ui->listWidget->addItem(tr("x01pr=\t (%1, %2, %3)").arg(list[2].x()).arg(list[2].y()).arg(list[2].z()));
    ui->listWidget->addItem(tr("x02pr=\t (%1, %2, %3)").arg(list[3].x()).arg(list[3].y()).arg(list[3].z()));
    ui->listWidget->addItem(tr("x02=\t (%1, %2, %3)").arg(list[4].x()).arg(list[4].y()).arg(list[4].z()));
    ui->listWidget->addItem(tr(""));

    ui->listWidget->addItem(tr("xP=\t (%1, %2, %3)").arg(list[5].x()).arg(list[5].y()).arg(list[5].z()));
    ui->listWidget->addItem(tr("xR=\t (%1, %2, %3)").arg(list[6].x()).arg(list[6].y()).arg(list[6].z()));
    ui->listWidget->addItem(tr("xT=\t (%1, %2, %3)").arg(jointPoints->GetToolPoint().x()).arg(jointPoints->GetToolPoint().y()).arg(jointPoints->GetToolPoint().z()));

    ui->listWidget->addItem(tr(""));

    //calculated joint points
    QList <QVector3D> list2 = jointPoints->GetCalculatedJointPoints();

    ui->listWidget->addItem(tr("xPcalc=\t (%1, %2, %3)").arg(list2[0].x()).arg(list2[0].y()).arg(list2[0].z()));
    ui->listWidget->addItem(tr("xRcalc=\t (%1, %2, %3)").arg(list2[1].x()).arg(list2[1].y()).arg(list2[1].z()));
    ui->listWidget->addItem(tr("xTcalc=\t (%1, %2, %3)").arg(list2[2].x()).arg(list2[2].y()).arg(list2[2].z()));

    ui->lineEditL1->setText(tr("%1").arg(jointPoints->GetL(1)));
    ui->lineEditL2->setText(tr("%1").arg(jointPoints->GetL(2)));
    ui->lineEditL3->setText(tr("%1").arg(jointPoints->GetL(3)));
    ui->lineEditL4->setText(tr("%1").arg(jointPoints->GetL(4)));
    ui->lineEditL5->setText(tr("%1").arg(jointPoints->GetL(5)));
    ui->lineEditL6->setText(tr("%1").arg(jointPoints->GetL(6)));

    ui->lineEditD->setText(tr("%1").arg(jointPoints->GetD()));
    ui->lineEditE->setText(tr("%1").arg(jointPoints->GetE()));

    ui->lineEditPsi->setText(tr("%1").arg(jointPoints->GetPsi(), 0, 'g', 3));
    ui->lineEditTheta->setText(tr("%1").arg(jointPoints->GetTheta(), 0, 'g', 3));

    ui->lineEditD1->setText(tr("%1").arg(jointPoints->GetDelta1(), 0, 'g', 3));
    ui->lineEditD2->setText(tr("%1").arg(jointPoints->GetDelta2(), 0, 'g', 3));
    ui->lineEditD5->setText(tr("%1").arg(jointPoints->GetDelta5(), 0, 'g', 3));

    QPixmap pix = QPixmap(70,50);
    pix.fill(Qt::darkGreen);
    ui->labelValid->setPixmap(pix);

}

void WidgetSettings::UpdateParameters()
{
    jointPoints->SetL(ui->lineEditL1->text().toDouble(), 1);
    jointPoints->SetL(ui->lineEditL2->text().toDouble(), 2);
    jointPoints->SetL(ui->lineEditL3->text().toDouble(), 3);
    jointPoints->SetL(ui->lineEditL4->text().toDouble(), 4);
    jointPoints->SetL(ui->lineEditL5->text().toDouble(), 5);
    jointPoints->SetL(ui->lineEditL6->text().toDouble(), 6);

    jointPoints->SetD(ui->lineEditD->text().toDouble());
    jointPoints->SetE(ui->lineEditE->text().toDouble());

    jointPoints->SetPsi(ui->lineEditPsi->text().toDouble());
    jointPoints->SetTheta(ui->lineEditTheta->text().toDouble());

    jointPoints->SetDelta1(ui->lineEditD1->text().toDouble());
    jointPoints->SetDelta2(ui->lineEditD2->text().toDouble());
    jointPoints->SetDelta5(ui->lineEditD5->text().toDouble());

    jointPoints->CalculateMachineCoordinates(jointPoints->GetToolPoint());

}

void WidgetSettings::OutOfRangeError()
{
    QPixmap pix = QPixmap(70,50);
    pix.fill(Qt::darkRed);
    ui->labelValid->setPixmap(pix);
}

void WidgetSettings::on_buttonReset_clicked()
{
    jointPoints->RestoreCustomSettings();
}
