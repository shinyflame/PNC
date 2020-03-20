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
#include <QtWidgets/QDial>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QProgressBar>
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
    QPushButton *pushButton_2;
    QPushButton *auto_2;
    QLineEdit *trajectory_num;
    QLineEdit *traffic_light;
    QLineEdit *PlanningTime;
    QLabel *label;
    QTextEdit *VehicleState;
    QDial *steering;
    QProgressBar *speed;
    QLineEdit *station;
    QPushButton *CleanMode;
    QPushButton *ParkingMode;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1466, 858);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(1330, 50, 89, 25));
        widget = new QCustomPlot(centralWidget);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(10, 0, 1091, 841));
        pushButton_2 = new QPushButton(centralWidget);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setGeometry(QRect(1330, 90, 89, 25));
        auto_2 = new QPushButton(centralWidget);
        auto_2->setObjectName(QStringLiteral("auto_2"));
        auto_2->setGeometry(QRect(1330, 130, 89, 25));
        trajectory_num = new QLineEdit(centralWidget);
        trajectory_num->setObjectName(QStringLiteral("trajectory_num"));
        trajectory_num->setGeometry(QRect(1252, 210, 191, 25));
        traffic_light = new QLineEdit(centralWidget);
        traffic_light->setObjectName(QStringLiteral("traffic_light"));
        traffic_light->setGeometry(QRect(1330, 170, 113, 25));
        PlanningTime = new QLineEdit(centralWidget);
        PlanningTime->setObjectName(QStringLiteral("PlanningTime"));
        PlanningTime->setGeometry(QRect(1330, 460, 113, 25));
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(1330, 430, 111, 20));
        VehicleState = new QTextEdit(centralWidget);
        VehicleState->setObjectName(QStringLiteral("VehicleState"));
        VehicleState->setGeometry(QRect(1280, 310, 161, 101));
        steering = new QDial(centralWidget);
        steering->setObjectName(QStringLiteral("steering"));
        steering->setGeometry(QRect(1190, 520, 261, 271));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(steering->sizePolicy().hasHeightForWidth());
        steering->setSizePolicy(sizePolicy);
        steering->setCursor(QCursor(Qt::UpArrowCursor));
        steering->setMouseTracking(false);
        steering->setTabletTracking(false);
        steering->setFocusPolicy(Qt::ClickFocus);
        steering->setAcceptDrops(true);
        steering->setAutoFillBackground(true);
        steering->setStyleSheet(QStringLiteral("color: rgb(138, 226, 52);"));
        steering->setInputMethodHints(Qt::ImhTime);
        steering->setMinimum(-180);
        steering->setMaximum(180);
        steering->setSingleStep(5);
        steering->setPageStep(90);
        steering->setValue(0);
        steering->setTracking(true);
        steering->setOrientation(Qt::Vertical);
        steering->setInvertedAppearance(false);
        steering->setInvertedControls(false);
        steering->setWrapping(true);
        steering->setNotchesVisible(true);
        speed = new QProgressBar(centralWidget);
        speed->setObjectName(QStringLiteral("speed"));
        speed->setGeometry(QRect(1190, 210, 31, 281));
        speed->setValue(24);
        speed->setOrientation(Qt::Vertical);
        station = new QLineEdit(centralWidget);
        station->setObjectName(QStringLiteral("station"));
        station->setGeometry(QRect(1252, 250, 191, 25));
        CleanMode = new QPushButton(centralWidget);
        CleanMode->setObjectName(QStringLiteral("CleanMode"));
        CleanMode->setGeometry(QRect(1210, 50, 89, 25));
        ParkingMode = new QPushButton(centralWidget);
        ParkingMode->setObjectName(QStringLiteral("ParkingMode"));
        ParkingMode->setGeometry(QRect(1210, 90, 89, 25));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1466, 22));
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
        pushButton_2->setText(QApplication::translate("MainWindow", "backward", nullptr));
        auto_2->setText(QApplication::translate("MainWindow", "auto", nullptr));
        label->setText(QApplication::translate("MainWindow", "CostTime:", nullptr));
        CleanMode->setText(QApplication::translate("MainWindow", "BorderClean", nullptr));
        ParkingMode->setText(QApplication::translate("MainWindow", "UnAutoPark", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
