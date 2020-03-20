/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.10.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include <qcustomplot.h>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *pushButton;
    QCustomPlot *widget;
    QPushButton *auto_2;
    QLineEdit *LocationStatusValue;
    QLabel *LocationStatusLabel;
    QTextEdit *VehicleState;
    QLabel *ProjectionStatusLabel;
    QLineEdit *ProjectionStatusValue;
    QLabel *WidthStatusLabel;
    QLineEdit *WidthStatusValue;
    QLabel *RouteStatusLabel;
    QLineEdit *RouteStatusValue;
    QLabel *ModeLabel;
    QLineEdit *ModeValue;
    QLabel *IntersectionStatusLabel;
    QLineEdit *IntersectionStatusValue;
    QLabel *SwitchStatusLabel;
    QLineEdit *SwitchStatusValue;
    QLabel *FinalStatusLabel;
    QLineEdit *FinalStatusValue;
    QLabel *CurrentMapIdLabel;
    QLineEdit *CurrentMapIdValue;
    QLabel *CurrentRoutesNumLabel;
    QLineEdit *CurrentRoutesNumValue;
    QLabel *CurrentRoutesInfoLabel;
    QLabel *CurrentSceneIdLabel;
    QLineEdit *CurrentSceneIdValue;
    QTextEdit *CurrentRoutesInfoValue;
    QLabel *FileLoadStatusLabel;
    QLineEdit *FileLoadStatusValue;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1502, 889);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(1170, 20, 89, 25));
        widget = new QCustomPlot(centralWidget);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(0, 0, 1141, 841));
        auto_2 = new QPushButton(centralWidget);
        auto_2->setObjectName(QStringLiteral("auto_2"));
        auto_2->setGeometry(QRect(1280, 20, 89, 25));
        LocationStatusValue = new QLineEdit(centralWidget);
        LocationStatusValue->setObjectName(QStringLiteral("LocationStatusValue"));
        LocationStatusValue->setGeometry(QRect(1170, 290, 150, 25));
        LocationStatusLabel = new QLabel(centralWidget);
        LocationStatusLabel->setObjectName(QStringLiteral("LocationStatusLabel"));
        LocationStatusLabel->setGeometry(QRect(1170, 270, 111, 20));
        VehicleState = new QTextEdit(centralWidget);
        VehicleState->setObjectName(QStringLiteral("VehicleState"));
        VehicleState->setGeometry(QRect(1170, 100, 161, 101));
        ProjectionStatusLabel = new QLabel(centralWidget);
        ProjectionStatusLabel->setObjectName(QStringLiteral("ProjectionStatusLabel"));
        ProjectionStatusLabel->setGeometry(QRect(1170, 330, 121, 20));
        ProjectionStatusValue = new QLineEdit(centralWidget);
        ProjectionStatusValue->setObjectName(QStringLiteral("ProjectionStatusValue"));
        ProjectionStatusValue->setGeometry(QRect(1170, 350, 150, 25));
        WidthStatusLabel = new QLabel(centralWidget);
        WidthStatusLabel->setObjectName(QStringLiteral("WidthStatusLabel"));
        WidthStatusLabel->setGeometry(QRect(1170, 390, 121, 20));
        WidthStatusValue = new QLineEdit(centralWidget);
        WidthStatusValue->setObjectName(QStringLiteral("WidthStatusValue"));
        WidthStatusValue->setGeometry(QRect(1170, 410, 311, 25));
        RouteStatusLabel = new QLabel(centralWidget);
        RouteStatusLabel->setObjectName(QStringLiteral("RouteStatusLabel"));
        RouteStatusLabel->setGeometry(QRect(1340, 210, 121, 20));
        RouteStatusValue = new QLineEdit(centralWidget);
        RouteStatusValue->setObjectName(QStringLiteral("RouteStatusValue"));
        RouteStatusValue->setGeometry(QRect(1340, 230, 150, 25));
        ModeLabel = new QLabel(centralWidget);
        ModeLabel->setObjectName(QStringLiteral("ModeLabel"));
        ModeLabel->setGeometry(QRect(1170, 210, 121, 20));
        ModeValue = new QLineEdit(centralWidget);
        ModeValue->setObjectName(QStringLiteral("ModeValue"));
        ModeValue->setGeometry(QRect(1170, 230, 150, 25));
        IntersectionStatusLabel = new QLabel(centralWidget);
        IntersectionStatusLabel->setObjectName(QStringLiteral("IntersectionStatusLabel"));
        IntersectionStatusLabel->setGeometry(QRect(1340, 270, 131, 20));
        IntersectionStatusValue = new QLineEdit(centralWidget);
        IntersectionStatusValue->setObjectName(QStringLiteral("IntersectionStatusValue"));
        IntersectionStatusValue->setGeometry(QRect(1340, 290, 150, 25));
        SwitchStatusLabel = new QLabel(centralWidget);
        SwitchStatusLabel->setObjectName(QStringLiteral("SwitchStatusLabel"));
        SwitchStatusLabel->setGeometry(QRect(1340, 330, 121, 20));
        SwitchStatusValue = new QLineEdit(centralWidget);
        SwitchStatusValue->setObjectName(QStringLiteral("SwitchStatusValue"));
        SwitchStatusValue->setGeometry(QRect(1340, 350, 150, 25));
        FinalStatusLabel = new QLabel(centralWidget);
        FinalStatusLabel->setObjectName(QStringLiteral("FinalStatusLabel"));
        FinalStatusLabel->setGeometry(QRect(1170, 450, 121, 20));
        FinalStatusValue = new QLineEdit(centralWidget);
        FinalStatusValue->setObjectName(QStringLiteral("FinalStatusValue"));
        FinalStatusValue->setGeometry(QRect(1170, 470, 150, 25));
        CurrentMapIdLabel = new QLabel(centralWidget);
        CurrentMapIdLabel->setObjectName(QStringLiteral("CurrentMapIdLabel"));
        CurrentMapIdLabel->setGeometry(QRect(1170, 590, 131, 20));
        CurrentMapIdValue = new QLineEdit(centralWidget);
        CurrentMapIdValue->setObjectName(QStringLiteral("CurrentMapIdValue"));
        CurrentMapIdValue->setGeometry(QRect(1170, 610, 140, 25));
        CurrentRoutesNumLabel = new QLabel(centralWidget);
        CurrentRoutesNumLabel->setObjectName(QStringLiteral("CurrentRoutesNumLabel"));
        CurrentRoutesNumLabel->setGeometry(QRect(1170, 650, 140, 20));
        CurrentRoutesNumValue = new QLineEdit(centralWidget);
        CurrentRoutesNumValue->setObjectName(QStringLiteral("CurrentRoutesNumValue"));
        CurrentRoutesNumValue->setGeometry(QRect(1170, 670, 140, 25));
        CurrentRoutesInfoLabel = new QLabel(centralWidget);
        CurrentRoutesInfoLabel->setObjectName(QStringLiteral("CurrentRoutesInfoLabel"));
        CurrentRoutesInfoLabel->setGeometry(QRect(1330, 530, 141, 20));
        CurrentSceneIdLabel = new QLabel(centralWidget);
        CurrentSceneIdLabel->setObjectName(QStringLiteral("CurrentSceneIdLabel"));
        CurrentSceneIdLabel->setGeometry(QRect(1170, 530, 131, 20));
        CurrentSceneIdValue = new QLineEdit(centralWidget);
        CurrentSceneIdValue->setObjectName(QStringLiteral("CurrentSceneIdValue"));
        CurrentSceneIdValue->setGeometry(QRect(1170, 550, 140, 25));
        CurrentRoutesInfoValue = new QTextEdit(centralWidget);
        CurrentRoutesInfoValue->setObjectName(QStringLiteral("CurrentRoutesInfoValue"));
        CurrentRoutesInfoValue->setGeometry(QRect(1330, 550, 161, 231));
        FileLoadStatusLabel = new QLabel(centralWidget);
        FileLoadStatusLabel->setObjectName(QStringLiteral("FileLoadStatusLabel"));
        FileLoadStatusLabel->setGeometry(QRect(1170, 50, 121, 20));
        FileLoadStatusValue = new QLineEdit(centralWidget);
        FileLoadStatusValue->setObjectName(QStringLiteral("FileLoadStatusValue"));
        FileLoadStatusValue->setGeometry(QRect(1170, 70, 201, 25));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1502, 22));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", nullptr));
        pushButton->setText(QApplication::translate("MainWindow", "forward", nullptr));
        auto_2->setText(QApplication::translate("MainWindow", "auto", nullptr));
        LocationStatusLabel->setText(QApplication::translate("MainWindow", "LocationStatus:", nullptr));
        ProjectionStatusLabel->setText(QApplication::translate("MainWindow", "ProjectionStatus:", nullptr));
        WidthStatusLabel->setText(QApplication::translate("MainWindow", "WidthStatus:", nullptr));
        RouteStatusLabel->setText(QApplication::translate("MainWindow", "RouteStatus:", nullptr));
        ModeLabel->setText(QApplication::translate("MainWindow", "Mode:", nullptr));
        IntersectionStatusLabel->setText(QApplication::translate("MainWindow", "IntersectionStatus:", nullptr));
        SwitchStatusLabel->setText(QApplication::translate("MainWindow", "SwitchStatus:", nullptr));
        FinalStatusLabel->setText(QApplication::translate("MainWindow", "FinalStatus:", nullptr));
        CurrentMapIdLabel->setText(QApplication::translate("MainWindow", "Current Map Id:", nullptr));
        CurrentRoutesNumLabel->setText(QApplication::translate("MainWindow", "Current Routes Num:", nullptr));
        CurrentRoutesInfoLabel->setText(QApplication::translate("MainWindow", "Current Routes Info:", nullptr));
        CurrentSceneIdLabel->setText(QApplication::translate("MainWindow", "Current Scene Id:", nullptr));
        FileLoadStatusLabel->setText(QApplication::translate("MainWindow", "FileLoadStatus:", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
