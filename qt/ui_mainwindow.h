/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Wed Apr 28 14:48:00 2010
**      by: Qt User Interface Compiler version 4.6.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDoubleSpinBox>
#include <QtGui/QFormLayout>
#include <QtGui/QFrame>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QPushButton>
#include <QtGui/QSpacerItem>
#include <QtGui/QSplitter>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QWidget>
#include "qwt_plot.h"
#include "qwt_slider.h"
#include "viewer.h"

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *action_Open;
    QAction *action_Close;
    QAction *actionAbout_Soft_Motion_Planner;
    QAction *actionFull_screen;
    QAction *actionHelp;
    QAction *action_Close_2;
    QWidget *centralWidget;
    QVBoxLayout *verticalLayout_17;
    QTabWidget *tabWidget_2;
    QWidget *tabSoftMotionPlanner;
    QVBoxLayout *verticalLayout_33;
    QWidget *widget_4;
    QHBoxLayout *horizontalLayout_7;
    QWidget *widget_19;
    QHBoxLayout *horizontalLayout_3;
    QwtPlot *qwtPlot;
    QwtPlot *qwtPlot_2;
    QFrame *frame_6;
    QVBoxLayout *verticalLayout_16;
    QLabel *label_29;
    QLabel *label_28;
    QWidget *widget_18;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_8;
    QDoubleSpinBox *doubleSpinBox_dc;
    QWidget *widget_20;
    QHBoxLayout *horizontalLayout_2;
    QWidget *widget_7;
    QVBoxLayout *verticalLayout_32;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QLabel *label_13;
    QLabel *label_14;
    QLabel *label_15;
    QWidget *widget_6;
    QVBoxLayout *verticalLayout_19;
    QDoubleSpinBox *doubleSpinBox_T1;
    QDoubleSpinBox *doubleSpinBox_T2;
    QDoubleSpinBox *doubleSpinBox_T3;
    QDoubleSpinBox *doubleSpinBox_T4;
    QDoubleSpinBox *doubleSpinBox_T5;
    QDoubleSpinBox *doubleSpinBox_T6;
    QDoubleSpinBox *doubleSpinBox_T7;
    QWidget *widget_21;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_30;
    QDoubleSpinBox *doubleSpinBox_totalTime;
    QWidget *widget_5;
    QHBoxLayout *horizontalLayout_4;
    QFrame *frame_4;
    QVBoxLayout *verticalLayout_24;
    QLabel *label_20;
    QWidget *widget_11;
    QVBoxLayout *verticalLayout_25;
    QLabel *label_21;
    QFormLayout *formLayout_9;
    QDoubleSpinBox *doubleSpinBox_Jmax_3;
    QwtSlider *Slider_Jmax_3;
    QWidget *widget_12;
    QVBoxLayout *verticalLayout_26;
    QLabel *label_22;
    QFormLayout *formLayout_10;
    QDoubleSpinBox *doubleSpinBox_Amax_3;
    QwtSlider *Slider_Amax_3;
    QWidget *widget_13;
    QVBoxLayout *verticalLayout_27;
    QLabel *label_23;
    QFormLayout *formLayout_11;
    QDoubleSpinBox *doubleSpinBox_Vmax_3;
    QwtSlider *Slider_Vmax_3;
    QFrame *frame_5;
    QVBoxLayout *verticalLayout_28;
    QLabel *label_24;
    QWidget *widget_15;
    QVBoxLayout *verticalLayout_29;
    QLabel *label_25;
    QFormLayout *formLayout_12;
    QDoubleSpinBox *doubleSpinBox_A0;
    QwtSlider *Slider_A0;
    QWidget *widget_16;
    QVBoxLayout *verticalLayout_30;
    QLabel *label_26;
    QFormLayout *formLayout_13;
    QDoubleSpinBox *doubleSpinBox_V0;
    QwtSlider *Slider_V0;
    QWidget *widget_17;
    QVBoxLayout *verticalLayout_31;
    QLabel *label_27;
    QFormLayout *formLayout_14;
    QDoubleSpinBox *doubleSpinBox_X0;
    QwtSlider *Slider_X0;
    QFrame *frame_3;
    QVBoxLayout *verticalLayout_20;
    QLabel *label_16;
    QWidget *widget_8;
    QVBoxLayout *verticalLayout_21;
    QLabel *label_17;
    QFormLayout *formLayout_5;
    QDoubleSpinBox *doubleSpinBox_Af;
    QwtSlider *Slider_Af;
    QWidget *widget_9;
    QVBoxLayout *verticalLayout_22;
    QLabel *label_18;
    QFormLayout *formLayout_7;
    QDoubleSpinBox *doubleSpinBox_Vf;
    QwtSlider *Slider_Vf;
    QWidget *widget_10;
    QVBoxLayout *verticalLayout_23;
    QLabel *label_19;
    QFormLayout *formLayout_8;
    QDoubleSpinBox *doubleSpinBox_Xf;
    QwtSlider *Slider_Xf;
    QWidget *tabTrajectoryApproximation;
    QHBoxLayout *horizontalLayout;
    QSplitter *splitter;
    QTabWidget *tabWidget;
    QWidget *planner;
    QVBoxLayout *verticalLayout_12;
    QFrame *frame;
    QVBoxLayout *verticalLayout_10;
    QLabel *label_4;
    QWidget *widget;
    QVBoxLayout *verticalLayout_7;
    QLabel *label;
    QFormLayout *formLayout;
    QDoubleSpinBox *doubleSpinBox_Jmax;
    QwtSlider *Slider_Jmax;
    QWidget *widget_2;
    QVBoxLayout *verticalLayout_8;
    QLabel *label_2;
    QFormLayout *formLayout_2;
    QDoubleSpinBox *doubleSpinBox_Amax;
    QwtSlider *Slider_Amax;
    QWidget *widget_3;
    QVBoxLayout *verticalLayout_9;
    QLabel *label_3;
    QFormLayout *formLayout_3;
    QDoubleSpinBox *doubleSpinBox_Vmax;
    QwtSlider *Slider_Vmax;
    QFrame *frame_2;
    QVBoxLayout *verticalLayout_11;
    QLabel *label_5;
    QVBoxLayout *verticalLayout_13;
    QLabel *label_6;
    QFormLayout *formLayout_4;
    QwtSlider *Slider_Interval;
    QDoubleSpinBox *doubleSpinBox;
    QVBoxLayout *verticalLayout_14;
    QLabel *label_7;
    QFormLayout *formLayout_6;
    QDoubleSpinBox *doubleSpinBox_SamplingTime;
    QwtSlider *Slider_SamplingTime;
    QSpacerItem *verticalSpacer;
    QPushButton *pushButton;
    QWidget *Traj;
    QVBoxLayout *verticalLayout_4;
    QwtPlot *qwtPlot_TrajJerk;
    QwtPlot *qwtPlot_TrajAcc;
    QwtPlot *qwtPlot_TrajVel;
    QWidget *axisX;
    QVBoxLayout *verticalLayout_3;
    QwtPlot *qwtPlot_JerkXapprox;
    QwtPlot *qwtPlot_AccXapprox;
    QwtPlot *qwtPlot_VelXapprox;
    QWidget *axisY;
    QVBoxLayout *verticalLayout_5;
    QwtPlot *qwtPlot_JerkYapprox;
    QwtPlot *qwtPlot_AccYapprox;
    QwtPlot *qwtPlot_VelYapprox;
    QWidget *axisZ;
    QVBoxLayout *verticalLayout_6;
    QwtPlot *qwtPlot_JerkZapprox;
    QwtPlot *qwtPlot_AccZapprox;
    QwtPlot *qwtPlot_VelZapprox;
    QWidget *tabIdealX;
    QVBoxLayout *verticalLayout;
    QwtPlot *qwtPlot_AccXideal;
    QwtPlot *qwtPlot_VelXideal;
    QwtPlot *qwtPlot_PosXideal;
    QWidget *tabIdealY;
    QVBoxLayout *verticalLayout_15;
    QwtPlot *qwtPlot_AccYideal;
    QwtPlot *qwtPlot_VelYideal;
    QwtPlot *qwtPlot_PosYideal;
    QWidget *tabError;
    QVBoxLayout *verticalLayout_2;
    QwtPlot *qwtPlot_haussdorff1;
    QwtPlot *qwtPlot_haussdorff2;
    QwtPlot *qwtPlot_errortraj;
    QWidget *tab_2;
    Viewer *viewer;
    QMenuBar *menuBar;
    QMenu *menu_File;
    QMenu *menu_Edit;
    QMenu *menu_Window;
    QMenu *menu_Help;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;
    QToolBar *toolBar;
    QToolBar *toolBar_2;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(979, 959);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(MainWindow->sizePolicy().hasHeightForWidth());
        MainWindow->setSizePolicy(sizePolicy);
        action_Open = new QAction(MainWindow);
        action_Open->setObjectName(QString::fromUtf8("action_Open"));
        action_Close = new QAction(MainWindow);
        action_Close->setObjectName(QString::fromUtf8("action_Close"));
        actionAbout_Soft_Motion_Planner = new QAction(MainWindow);
        actionAbout_Soft_Motion_Planner->setObjectName(QString::fromUtf8("actionAbout_Soft_Motion_Planner"));
        actionFull_screen = new QAction(MainWindow);
        actionFull_screen->setObjectName(QString::fromUtf8("actionFull_screen"));
        actionHelp = new QAction(MainWindow);
        actionHelp->setObjectName(QString::fromUtf8("actionHelp"));
        action_Close_2 = new QAction(MainWindow);
        action_Close_2->setObjectName(QString::fromUtf8("action_Close_2"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        QSizePolicy sizePolicy1(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(centralWidget->sizePolicy().hasHeightForWidth());
        centralWidget->setSizePolicy(sizePolicy1);
        verticalLayout_17 = new QVBoxLayout(centralWidget);
        verticalLayout_17->setSpacing(6);
        verticalLayout_17->setContentsMargins(11, 11, 11, 11);
        verticalLayout_17->setObjectName(QString::fromUtf8("verticalLayout_17"));
        tabWidget_2 = new QTabWidget(centralWidget);
        tabWidget_2->setObjectName(QString::fromUtf8("tabWidget_2"));
        sizePolicy1.setHeightForWidth(tabWidget_2->sizePolicy().hasHeightForWidth());
        tabWidget_2->setSizePolicy(sizePolicy1);
        tabSoftMotionPlanner = new QWidget();
        tabSoftMotionPlanner->setObjectName(QString::fromUtf8("tabSoftMotionPlanner"));
        verticalLayout_33 = new QVBoxLayout(tabSoftMotionPlanner);
        verticalLayout_33->setSpacing(6);
        verticalLayout_33->setContentsMargins(11, 11, 11, 11);
        verticalLayout_33->setObjectName(QString::fromUtf8("verticalLayout_33"));
        widget_4 = new QWidget(tabSoftMotionPlanner);
        widget_4->setObjectName(QString::fromUtf8("widget_4"));
        horizontalLayout_7 = new QHBoxLayout(widget_4);
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_7->setObjectName(QString::fromUtf8("horizontalLayout_7"));
        widget_19 = new QWidget(widget_4);
        widget_19->setObjectName(QString::fromUtf8("widget_19"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Expanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(widget_19->sizePolicy().hasHeightForWidth());
        widget_19->setSizePolicy(sizePolicy2);
        horizontalLayout_3 = new QHBoxLayout(widget_19);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QString::fromUtf8("horizontalLayout_3"));
        qwtPlot = new QwtPlot(widget_19);
        qwtPlot->setObjectName(QString::fromUtf8("qwtPlot"));
        QSizePolicy sizePolicy3(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(qwtPlot->sizePolicy().hasHeightForWidth());
        qwtPlot->setSizePolicy(sizePolicy3);

        horizontalLayout_3->addWidget(qwtPlot);

        qwtPlot_2 = new QwtPlot(widget_19);
        qwtPlot_2->setObjectName(QString::fromUtf8("qwtPlot_2"));
        sizePolicy3.setHeightForWidth(qwtPlot_2->sizePolicy().hasHeightForWidth());
        qwtPlot_2->setSizePolicy(sizePolicy3);

        horizontalLayout_3->addWidget(qwtPlot_2);


        horizontalLayout_7->addWidget(widget_19);

        frame_6 = new QFrame(widget_4);
        frame_6->setObjectName(QString::fromUtf8("frame_6"));
        frame_6->setFrameShape(QFrame::Panel);
        frame_6->setFrameShadow(QFrame::Sunken);
        frame_6->setLineWidth(1);
        verticalLayout_16 = new QVBoxLayout(frame_6);
        verticalLayout_16->setSpacing(6);
        verticalLayout_16->setContentsMargins(11, 11, 11, 11);
        verticalLayout_16->setObjectName(QString::fromUtf8("verticalLayout_16"));
        label_29 = new QLabel(frame_6);
        label_29->setObjectName(QString::fromUtf8("label_29"));
        label_29->setPixmap(QPixmap(QString::fromUtf8("logoLaas.png")));
        label_29->setScaledContents(false);
        label_29->setAlignment(Qt::AlignCenter);

        verticalLayout_16->addWidget(label_29);

        label_28 = new QLabel(frame_6);
        label_28->setObjectName(QString::fromUtf8("label_28"));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        label_28->setFont(font);
        label_28->setLayoutDirection(Qt::LeftToRight);
        label_28->setAutoFillBackground(false);
        label_28->setAlignment(Qt::AlignCenter);

        verticalLayout_16->addWidget(label_28);

        widget_18 = new QWidget(frame_6);
        widget_18->setObjectName(QString::fromUtf8("widget_18"));
        horizontalLayout_5 = new QHBoxLayout(widget_18);
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_5->setObjectName(QString::fromUtf8("horizontalLayout_5"));
        label_8 = new QLabel(widget_18);
        label_8->setObjectName(QString::fromUtf8("label_8"));

        horizontalLayout_5->addWidget(label_8);

        doubleSpinBox_dc = new QDoubleSpinBox(widget_18);
        doubleSpinBox_dc->setObjectName(QString::fromUtf8("doubleSpinBox_dc"));
        doubleSpinBox_dc->setButtonSymbols(QAbstractSpinBox::NoButtons);
        doubleSpinBox_dc->setKeyboardTracking(true);
        doubleSpinBox_dc->setDecimals(6);
        doubleSpinBox_dc->setMinimum(-99);

        horizontalLayout_5->addWidget(doubleSpinBox_dc);


        verticalLayout_16->addWidget(widget_18);

        widget_20 = new QWidget(frame_6);
        widget_20->setObjectName(QString::fromUtf8("widget_20"));
        horizontalLayout_2 = new QHBoxLayout(widget_20);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        widget_7 = new QWidget(widget_20);
        widget_7->setObjectName(QString::fromUtf8("widget_7"));
        verticalLayout_32 = new QVBoxLayout(widget_7);
        verticalLayout_32->setSpacing(6);
        verticalLayout_32->setContentsMargins(11, 11, 11, 11);
        verticalLayout_32->setObjectName(QString::fromUtf8("verticalLayout_32"));
        label_9 = new QLabel(widget_7);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setScaledContents(false);

        verticalLayout_32->addWidget(label_9);

        label_10 = new QLabel(widget_7);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        verticalLayout_32->addWidget(label_10);

        label_11 = new QLabel(widget_7);
        label_11->setObjectName(QString::fromUtf8("label_11"));

        verticalLayout_32->addWidget(label_11);

        label_12 = new QLabel(widget_7);
        label_12->setObjectName(QString::fromUtf8("label_12"));

        verticalLayout_32->addWidget(label_12);

        label_13 = new QLabel(widget_7);
        label_13->setObjectName(QString::fromUtf8("label_13"));

        verticalLayout_32->addWidget(label_13);

        label_14 = new QLabel(widget_7);
        label_14->setObjectName(QString::fromUtf8("label_14"));

        verticalLayout_32->addWidget(label_14);

        label_15 = new QLabel(widget_7);
        label_15->setObjectName(QString::fromUtf8("label_15"));

        verticalLayout_32->addWidget(label_15);


        horizontalLayout_2->addWidget(widget_7);

        widget_6 = new QWidget(widget_20);
        widget_6->setObjectName(QString::fromUtf8("widget_6"));
        verticalLayout_19 = new QVBoxLayout(widget_6);
        verticalLayout_19->setSpacing(6);
        verticalLayout_19->setContentsMargins(11, 11, 11, 11);
        verticalLayout_19->setObjectName(QString::fromUtf8("verticalLayout_19"));
        doubleSpinBox_T1 = new QDoubleSpinBox(widget_6);
        doubleSpinBox_T1->setObjectName(QString::fromUtf8("doubleSpinBox_T1"));
        doubleSpinBox_T1->setButtonSymbols(QAbstractSpinBox::NoButtons);
        doubleSpinBox_T1->setKeyboardTracking(true);
        doubleSpinBox_T1->setDecimals(4);

        verticalLayout_19->addWidget(doubleSpinBox_T1);

        doubleSpinBox_T2 = new QDoubleSpinBox(widget_6);
        doubleSpinBox_T2->setObjectName(QString::fromUtf8("doubleSpinBox_T2"));
        doubleSpinBox_T2->setButtonSymbols(QAbstractSpinBox::NoButtons);
        doubleSpinBox_T2->setKeyboardTracking(true);
        doubleSpinBox_T2->setDecimals(4);

        verticalLayout_19->addWidget(doubleSpinBox_T2);

        doubleSpinBox_T3 = new QDoubleSpinBox(widget_6);
        doubleSpinBox_T3->setObjectName(QString::fromUtf8("doubleSpinBox_T3"));
        doubleSpinBox_T3->setButtonSymbols(QAbstractSpinBox::NoButtons);
        doubleSpinBox_T3->setKeyboardTracking(true);
        doubleSpinBox_T3->setDecimals(4);

        verticalLayout_19->addWidget(doubleSpinBox_T3);

        doubleSpinBox_T4 = new QDoubleSpinBox(widget_6);
        doubleSpinBox_T4->setObjectName(QString::fromUtf8("doubleSpinBox_T4"));
        doubleSpinBox_T4->setButtonSymbols(QAbstractSpinBox::NoButtons);
        doubleSpinBox_T4->setKeyboardTracking(true);
        doubleSpinBox_T4->setDecimals(4);

        verticalLayout_19->addWidget(doubleSpinBox_T4);

        doubleSpinBox_T5 = new QDoubleSpinBox(widget_6);
        doubleSpinBox_T5->setObjectName(QString::fromUtf8("doubleSpinBox_T5"));
        doubleSpinBox_T5->setButtonSymbols(QAbstractSpinBox::NoButtons);
        doubleSpinBox_T5->setKeyboardTracking(true);
        doubleSpinBox_T5->setDecimals(4);

        verticalLayout_19->addWidget(doubleSpinBox_T5);

        doubleSpinBox_T6 = new QDoubleSpinBox(widget_6);
        doubleSpinBox_T6->setObjectName(QString::fromUtf8("doubleSpinBox_T6"));
        doubleSpinBox_T6->setButtonSymbols(QAbstractSpinBox::NoButtons);
        doubleSpinBox_T6->setKeyboardTracking(true);
        doubleSpinBox_T6->setDecimals(4);

        verticalLayout_19->addWidget(doubleSpinBox_T6);

        doubleSpinBox_T7 = new QDoubleSpinBox(widget_6);
        doubleSpinBox_T7->setObjectName(QString::fromUtf8("doubleSpinBox_T7"));
        doubleSpinBox_T7->setButtonSymbols(QAbstractSpinBox::NoButtons);
        doubleSpinBox_T7->setKeyboardTracking(true);
        doubleSpinBox_T7->setDecimals(4);

        verticalLayout_19->addWidget(doubleSpinBox_T7);

        widget_7->raise();
        doubleSpinBox_T7->raise();
        doubleSpinBox_T7->raise();
        doubleSpinBox_T1->raise();
        doubleSpinBox_T2->raise();
        doubleSpinBox_T3->raise();
        doubleSpinBox_T4->raise();
        doubleSpinBox_T5->raise();
        doubleSpinBox_T6->raise();

        horizontalLayout_2->addWidget(widget_6);


        verticalLayout_16->addWidget(widget_20);

        widget_21 = new QWidget(frame_6);
        widget_21->setObjectName(QString::fromUtf8("widget_21"));
        horizontalLayout_6 = new QHBoxLayout(widget_21);
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_6->setObjectName(QString::fromUtf8("horizontalLayout_6"));
        label_30 = new QLabel(widget_21);
        label_30->setObjectName(QString::fromUtf8("label_30"));

        horizontalLayout_6->addWidget(label_30);

        doubleSpinBox_totalTime = new QDoubleSpinBox(widget_21);
        doubleSpinBox_totalTime->setObjectName(QString::fromUtf8("doubleSpinBox_totalTime"));
        doubleSpinBox_totalTime->setButtonSymbols(QAbstractSpinBox::NoButtons);
        doubleSpinBox_totalTime->setKeyboardTracking(true);
        doubleSpinBox_totalTime->setDecimals(6);
        doubleSpinBox_totalTime->setMinimum(-99);

        horizontalLayout_6->addWidget(doubleSpinBox_totalTime);


        verticalLayout_16->addWidget(widget_21);


        horizontalLayout_7->addWidget(frame_6);


        verticalLayout_33->addWidget(widget_4);

        widget_5 = new QWidget(tabSoftMotionPlanner);
        widget_5->setObjectName(QString::fromUtf8("widget_5"));
        horizontalLayout_4 = new QHBoxLayout(widget_5);
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_4->setObjectName(QString::fromUtf8("horizontalLayout_4"));
        frame_4 = new QFrame(widget_5);
        frame_4->setObjectName(QString::fromUtf8("frame_4"));
        QSizePolicy sizePolicy4(QSizePolicy::Preferred, QSizePolicy::Minimum);
        sizePolicy4.setHorizontalStretch(0);
        sizePolicy4.setVerticalStretch(0);
        sizePolicy4.setHeightForWidth(frame_4->sizePolicy().hasHeightForWidth());
        frame_4->setSizePolicy(sizePolicy4);
        frame_4->setFrameShape(QFrame::Panel);
        frame_4->setFrameShadow(QFrame::Sunken);
        frame_4->setLineWidth(1);
        verticalLayout_24 = new QVBoxLayout(frame_4);
        verticalLayout_24->setSpacing(6);
        verticalLayout_24->setContentsMargins(11, 11, 11, 11);
        verticalLayout_24->setObjectName(QString::fromUtf8("verticalLayout_24"));
        label_20 = new QLabel(frame_4);
        label_20->setObjectName(QString::fromUtf8("label_20"));
        QFont font1;
        font1.setBold(true);
        font1.setItalic(false);
        font1.setWeight(75);
        font1.setStrikeOut(false);
        label_20->setFont(font1);

        verticalLayout_24->addWidget(label_20);

        widget_11 = new QWidget(frame_4);
        widget_11->setObjectName(QString::fromUtf8("widget_11"));
        verticalLayout_25 = new QVBoxLayout(widget_11);
        verticalLayout_25->setSpacing(6);
        verticalLayout_25->setContentsMargins(11, 11, 11, 11);
        verticalLayout_25->setObjectName(QString::fromUtf8("verticalLayout_25"));
        label_21 = new QLabel(widget_11);
        label_21->setObjectName(QString::fromUtf8("label_21"));

        verticalLayout_25->addWidget(label_21);

        formLayout_9 = new QFormLayout();
        formLayout_9->setSpacing(6);
        formLayout_9->setObjectName(QString::fromUtf8("formLayout_9"));
        doubleSpinBox_Jmax_3 = new QDoubleSpinBox(widget_11);
        doubleSpinBox_Jmax_3->setObjectName(QString::fromUtf8("doubleSpinBox_Jmax_3"));
        doubleSpinBox_Jmax_3->setDecimals(3);
        doubleSpinBox_Jmax_3->setSingleStep(0.01);

        formLayout_9->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_Jmax_3);

        Slider_Jmax_3 = new QwtSlider(widget_11);
        Slider_Jmax_3->setObjectName(QString::fromUtf8("Slider_Jmax_3"));
        Slider_Jmax_3->setValid(true);
        Slider_Jmax_3->setMass(0);
        Slider_Jmax_3->setThumbLength(31);
        Slider_Jmax_3->setThumbWidth(16);

        formLayout_9->setWidget(0, QFormLayout::FieldRole, Slider_Jmax_3);


        verticalLayout_25->addLayout(formLayout_9);


        verticalLayout_24->addWidget(widget_11);

        widget_12 = new QWidget(frame_4);
        widget_12->setObjectName(QString::fromUtf8("widget_12"));
        verticalLayout_26 = new QVBoxLayout(widget_12);
        verticalLayout_26->setSpacing(6);
        verticalLayout_26->setContentsMargins(11, 11, 11, 11);
        verticalLayout_26->setObjectName(QString::fromUtf8("verticalLayout_26"));
        label_22 = new QLabel(widget_12);
        label_22->setObjectName(QString::fromUtf8("label_22"));

        verticalLayout_26->addWidget(label_22);

        formLayout_10 = new QFormLayout();
        formLayout_10->setSpacing(6);
        formLayout_10->setObjectName(QString::fromUtf8("formLayout_10"));
        doubleSpinBox_Amax_3 = new QDoubleSpinBox(widget_12);
        doubleSpinBox_Amax_3->setObjectName(QString::fromUtf8("doubleSpinBox_Amax_3"));
        doubleSpinBox_Amax_3->setDecimals(3);
        doubleSpinBox_Amax_3->setSingleStep(0.01);

        formLayout_10->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_Amax_3);

        Slider_Amax_3 = new QwtSlider(widget_12);
        Slider_Amax_3->setObjectName(QString::fromUtf8("Slider_Amax_3"));

        formLayout_10->setWidget(0, QFormLayout::FieldRole, Slider_Amax_3);


        verticalLayout_26->addLayout(formLayout_10);


        verticalLayout_24->addWidget(widget_12);

        widget_13 = new QWidget(frame_4);
        widget_13->setObjectName(QString::fromUtf8("widget_13"));
        verticalLayout_27 = new QVBoxLayout(widget_13);
        verticalLayout_27->setSpacing(6);
        verticalLayout_27->setContentsMargins(11, 11, 11, 11);
        verticalLayout_27->setObjectName(QString::fromUtf8("verticalLayout_27"));
        label_23 = new QLabel(widget_13);
        label_23->setObjectName(QString::fromUtf8("label_23"));

        verticalLayout_27->addWidget(label_23);

        formLayout_11 = new QFormLayout();
        formLayout_11->setSpacing(6);
        formLayout_11->setObjectName(QString::fromUtf8("formLayout_11"));
        doubleSpinBox_Vmax_3 = new QDoubleSpinBox(widget_13);
        doubleSpinBox_Vmax_3->setObjectName(QString::fromUtf8("doubleSpinBox_Vmax_3"));
        doubleSpinBox_Vmax_3->setDecimals(3);
        doubleSpinBox_Vmax_3->setSingleStep(0.01);

        formLayout_11->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_Vmax_3);

        Slider_Vmax_3 = new QwtSlider(widget_13);
        Slider_Vmax_3->setObjectName(QString::fromUtf8("Slider_Vmax_3"));

        formLayout_11->setWidget(0, QFormLayout::FieldRole, Slider_Vmax_3);


        verticalLayout_27->addLayout(formLayout_11);


        verticalLayout_24->addWidget(widget_13);


        horizontalLayout_4->addWidget(frame_4);

        frame_5 = new QFrame(widget_5);
        frame_5->setObjectName(QString::fromUtf8("frame_5"));
        sizePolicy4.setHeightForWidth(frame_5->sizePolicy().hasHeightForWidth());
        frame_5->setSizePolicy(sizePolicy4);
        frame_5->setFrameShape(QFrame::Panel);
        frame_5->setFrameShadow(QFrame::Sunken);
        frame_5->setLineWidth(1);
        verticalLayout_28 = new QVBoxLayout(frame_5);
        verticalLayout_28->setSpacing(6);
        verticalLayout_28->setContentsMargins(11, 11, 11, 11);
        verticalLayout_28->setObjectName(QString::fromUtf8("verticalLayout_28"));
        label_24 = new QLabel(frame_5);
        label_24->setObjectName(QString::fromUtf8("label_24"));
        label_24->setFont(font1);

        verticalLayout_28->addWidget(label_24);

        widget_15 = new QWidget(frame_5);
        widget_15->setObjectName(QString::fromUtf8("widget_15"));
        verticalLayout_29 = new QVBoxLayout(widget_15);
        verticalLayout_29->setSpacing(6);
        verticalLayout_29->setContentsMargins(11, 11, 11, 11);
        verticalLayout_29->setObjectName(QString::fromUtf8("verticalLayout_29"));
        label_25 = new QLabel(widget_15);
        label_25->setObjectName(QString::fromUtf8("label_25"));

        verticalLayout_29->addWidget(label_25);

        formLayout_12 = new QFormLayout();
        formLayout_12->setSpacing(6);
        formLayout_12->setObjectName(QString::fromUtf8("formLayout_12"));
        doubleSpinBox_A0 = new QDoubleSpinBox(widget_15);
        doubleSpinBox_A0->setObjectName(QString::fromUtf8("doubleSpinBox_A0"));
        doubleSpinBox_A0->setDecimals(3);
        doubleSpinBox_A0->setSingleStep(0.01);

        formLayout_12->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_A0);

        Slider_A0 = new QwtSlider(widget_15);
        Slider_A0->setObjectName(QString::fromUtf8("Slider_A0"));
        Slider_A0->setValid(true);
        Slider_A0->setMass(0);
        Slider_A0->setThumbLength(31);
        Slider_A0->setThumbWidth(16);

        formLayout_12->setWidget(0, QFormLayout::FieldRole, Slider_A0);


        verticalLayout_29->addLayout(formLayout_12);


        verticalLayout_28->addWidget(widget_15);

        widget_16 = new QWidget(frame_5);
        widget_16->setObjectName(QString::fromUtf8("widget_16"));
        verticalLayout_30 = new QVBoxLayout(widget_16);
        verticalLayout_30->setSpacing(6);
        verticalLayout_30->setContentsMargins(11, 11, 11, 11);
        verticalLayout_30->setObjectName(QString::fromUtf8("verticalLayout_30"));
        label_26 = new QLabel(widget_16);
        label_26->setObjectName(QString::fromUtf8("label_26"));

        verticalLayout_30->addWidget(label_26);

        formLayout_13 = new QFormLayout();
        formLayout_13->setSpacing(6);
        formLayout_13->setObjectName(QString::fromUtf8("formLayout_13"));
        doubleSpinBox_V0 = new QDoubleSpinBox(widget_16);
        doubleSpinBox_V0->setObjectName(QString::fromUtf8("doubleSpinBox_V0"));
        doubleSpinBox_V0->setDecimals(3);
        doubleSpinBox_V0->setSingleStep(0.01);

        formLayout_13->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_V0);

        Slider_V0 = new QwtSlider(widget_16);
        Slider_V0->setObjectName(QString::fromUtf8("Slider_V0"));

        formLayout_13->setWidget(0, QFormLayout::FieldRole, Slider_V0);


        verticalLayout_30->addLayout(formLayout_13);


        verticalLayout_28->addWidget(widget_16);

        widget_17 = new QWidget(frame_5);
        widget_17->setObjectName(QString::fromUtf8("widget_17"));
        verticalLayout_31 = new QVBoxLayout(widget_17);
        verticalLayout_31->setSpacing(6);
        verticalLayout_31->setContentsMargins(11, 11, 11, 11);
        verticalLayout_31->setObjectName(QString::fromUtf8("verticalLayout_31"));
        label_27 = new QLabel(widget_17);
        label_27->setObjectName(QString::fromUtf8("label_27"));

        verticalLayout_31->addWidget(label_27);

        formLayout_14 = new QFormLayout();
        formLayout_14->setSpacing(6);
        formLayout_14->setObjectName(QString::fromUtf8("formLayout_14"));
        doubleSpinBox_X0 = new QDoubleSpinBox(widget_17);
        doubleSpinBox_X0->setObjectName(QString::fromUtf8("doubleSpinBox_X0"));
        doubleSpinBox_X0->setDecimals(3);
        doubleSpinBox_X0->setMaximum(0);
        doubleSpinBox_X0->setSingleStep(0.001);

        formLayout_14->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_X0);

        Slider_X0 = new QwtSlider(widget_17);
        Slider_X0->setObjectName(QString::fromUtf8("Slider_X0"));

        formLayout_14->setWidget(0, QFormLayout::FieldRole, Slider_X0);


        verticalLayout_31->addLayout(formLayout_14);


        verticalLayout_28->addWidget(widget_17);


        horizontalLayout_4->addWidget(frame_5);

        frame_3 = new QFrame(widget_5);
        frame_3->setObjectName(QString::fromUtf8("frame_3"));
        sizePolicy4.setHeightForWidth(frame_3->sizePolicy().hasHeightForWidth());
        frame_3->setSizePolicy(sizePolicy4);
        frame_3->setFrameShape(QFrame::Panel);
        frame_3->setFrameShadow(QFrame::Sunken);
        frame_3->setLineWidth(1);
        verticalLayout_20 = new QVBoxLayout(frame_3);
        verticalLayout_20->setSpacing(6);
        verticalLayout_20->setContentsMargins(11, 11, 11, 11);
        verticalLayout_20->setObjectName(QString::fromUtf8("verticalLayout_20"));
        label_16 = new QLabel(frame_3);
        label_16->setObjectName(QString::fromUtf8("label_16"));
        label_16->setFont(font1);

        verticalLayout_20->addWidget(label_16);

        widget_8 = new QWidget(frame_3);
        widget_8->setObjectName(QString::fromUtf8("widget_8"));
        verticalLayout_21 = new QVBoxLayout(widget_8);
        verticalLayout_21->setSpacing(6);
        verticalLayout_21->setContentsMargins(11, 11, 11, 11);
        verticalLayout_21->setObjectName(QString::fromUtf8("verticalLayout_21"));
        label_17 = new QLabel(widget_8);
        label_17->setObjectName(QString::fromUtf8("label_17"));

        verticalLayout_21->addWidget(label_17);

        formLayout_5 = new QFormLayout();
        formLayout_5->setSpacing(6);
        formLayout_5->setObjectName(QString::fromUtf8("formLayout_5"));
        doubleSpinBox_Af = new QDoubleSpinBox(widget_8);
        doubleSpinBox_Af->setObjectName(QString::fromUtf8("doubleSpinBox_Af"));
        doubleSpinBox_Af->setDecimals(3);
        doubleSpinBox_Af->setSingleStep(0.01);

        formLayout_5->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_Af);

        Slider_Af = new QwtSlider(widget_8);
        Slider_Af->setObjectName(QString::fromUtf8("Slider_Af"));
        Slider_Af->setValid(true);
        Slider_Af->setMass(0);
        Slider_Af->setThumbLength(31);
        Slider_Af->setThumbWidth(16);

        formLayout_5->setWidget(0, QFormLayout::FieldRole, Slider_Af);


        verticalLayout_21->addLayout(formLayout_5);


        verticalLayout_20->addWidget(widget_8);

        widget_9 = new QWidget(frame_3);
        widget_9->setObjectName(QString::fromUtf8("widget_9"));
        verticalLayout_22 = new QVBoxLayout(widget_9);
        verticalLayout_22->setSpacing(6);
        verticalLayout_22->setContentsMargins(11, 11, 11, 11);
        verticalLayout_22->setObjectName(QString::fromUtf8("verticalLayout_22"));
        label_18 = new QLabel(widget_9);
        label_18->setObjectName(QString::fromUtf8("label_18"));

        verticalLayout_22->addWidget(label_18);

        formLayout_7 = new QFormLayout();
        formLayout_7->setSpacing(6);
        formLayout_7->setObjectName(QString::fromUtf8("formLayout_7"));
        doubleSpinBox_Vf = new QDoubleSpinBox(widget_9);
        doubleSpinBox_Vf->setObjectName(QString::fromUtf8("doubleSpinBox_Vf"));
        doubleSpinBox_Vf->setDecimals(3);
        doubleSpinBox_Vf->setSingleStep(0.01);

        formLayout_7->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_Vf);

        Slider_Vf = new QwtSlider(widget_9);
        Slider_Vf->setObjectName(QString::fromUtf8("Slider_Vf"));

        formLayout_7->setWidget(0, QFormLayout::FieldRole, Slider_Vf);


        verticalLayout_22->addLayout(formLayout_7);


        verticalLayout_20->addWidget(widget_9);

        widget_10 = new QWidget(frame_3);
        widget_10->setObjectName(QString::fromUtf8("widget_10"));
        verticalLayout_23 = new QVBoxLayout(widget_10);
        verticalLayout_23->setSpacing(6);
        verticalLayout_23->setContentsMargins(11, 11, 11, 11);
        verticalLayout_23->setObjectName(QString::fromUtf8("verticalLayout_23"));
        label_19 = new QLabel(widget_10);
        label_19->setObjectName(QString::fromUtf8("label_19"));

        verticalLayout_23->addWidget(label_19);

        formLayout_8 = new QFormLayout();
        formLayout_8->setSpacing(6);
        formLayout_8->setObjectName(QString::fromUtf8("formLayout_8"));
        doubleSpinBox_Xf = new QDoubleSpinBox(widget_10);
        doubleSpinBox_Xf->setObjectName(QString::fromUtf8("doubleSpinBox_Xf"));
        doubleSpinBox_Xf->setDecimals(3);
        doubleSpinBox_Xf->setSingleStep(0.001);

        formLayout_8->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_Xf);

        Slider_Xf = new QwtSlider(widget_10);
        Slider_Xf->setObjectName(QString::fromUtf8("Slider_Xf"));

        formLayout_8->setWidget(0, QFormLayout::FieldRole, Slider_Xf);


        verticalLayout_23->addLayout(formLayout_8);


        verticalLayout_20->addWidget(widget_10);


        horizontalLayout_4->addWidget(frame_3);


        verticalLayout_33->addWidget(widget_5);

        tabWidget_2->addTab(tabSoftMotionPlanner, QString());
        tabTrajectoryApproximation = new QWidget();
        tabTrajectoryApproximation->setObjectName(QString::fromUtf8("tabTrajectoryApproximation"));
        horizontalLayout = new QHBoxLayout(tabTrajectoryApproximation);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        splitter = new QSplitter(tabTrajectoryApproximation);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setOrientation(Qt::Horizontal);
        tabWidget = new QTabWidget(splitter);
        tabWidget->setObjectName(QString::fromUtf8("tabWidget"));
        QSizePolicy sizePolicy5(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy5.setHorizontalStretch(35);
        sizePolicy5.setVerticalStretch(0);
        sizePolicy5.setHeightForWidth(tabWidget->sizePolicy().hasHeightForWidth());
        tabWidget->setSizePolicy(sizePolicy5);
        tabWidget->setBaseSize(QSize(0, 0));
        QFont font2;
        font2.setItalic(false);
        tabWidget->setFont(font2);
        tabWidget->setTabPosition(QTabWidget::North);
        tabWidget->setTabShape(QTabWidget::Rounded);
        planner = new QWidget();
        planner->setObjectName(QString::fromUtf8("planner"));
        verticalLayout_12 = new QVBoxLayout(planner);
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setContentsMargins(11, 11, 11, 11);
        verticalLayout_12->setObjectName(QString::fromUtf8("verticalLayout_12"));
        frame = new QFrame(planner);
        frame->setObjectName(QString::fromUtf8("frame"));
        sizePolicy4.setHeightForWidth(frame->sizePolicy().hasHeightForWidth());
        frame->setSizePolicy(sizePolicy4);
        frame->setFrameShape(QFrame::Panel);
        frame->setFrameShadow(QFrame::Sunken);
        frame->setLineWidth(1);
        verticalLayout_10 = new QVBoxLayout(frame);
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setContentsMargins(11, 11, 11, 11);
        verticalLayout_10->setObjectName(QString::fromUtf8("verticalLayout_10"));
        label_4 = new QLabel(frame);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setFont(font1);

        verticalLayout_10->addWidget(label_4);

        widget = new QWidget(frame);
        widget->setObjectName(QString::fromUtf8("widget"));
        verticalLayout_7 = new QVBoxLayout(widget);
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setContentsMargins(11, 11, 11, 11);
        verticalLayout_7->setObjectName(QString::fromUtf8("verticalLayout_7"));
        label = new QLabel(widget);
        label->setObjectName(QString::fromUtf8("label"));

        verticalLayout_7->addWidget(label);

        formLayout = new QFormLayout();
        formLayout->setSpacing(6);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        doubleSpinBox_Jmax = new QDoubleSpinBox(widget);
        doubleSpinBox_Jmax->setObjectName(QString::fromUtf8("doubleSpinBox_Jmax"));

        formLayout->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_Jmax);

        Slider_Jmax = new QwtSlider(widget);
        Slider_Jmax->setObjectName(QString::fromUtf8("Slider_Jmax"));
        Slider_Jmax->setValid(true);
        Slider_Jmax->setMass(0);
        Slider_Jmax->setThumbLength(31);
        Slider_Jmax->setThumbWidth(16);

        formLayout->setWidget(0, QFormLayout::FieldRole, Slider_Jmax);


        verticalLayout_7->addLayout(formLayout);


        verticalLayout_10->addWidget(widget);

        widget_2 = new QWidget(frame);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        verticalLayout_8 = new QVBoxLayout(widget_2);
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setContentsMargins(11, 11, 11, 11);
        verticalLayout_8->setObjectName(QString::fromUtf8("verticalLayout_8"));
        label_2 = new QLabel(widget_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        verticalLayout_8->addWidget(label_2);

        formLayout_2 = new QFormLayout();
        formLayout_2->setSpacing(6);
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        doubleSpinBox_Amax = new QDoubleSpinBox(widget_2);
        doubleSpinBox_Amax->setObjectName(QString::fromUtf8("doubleSpinBox_Amax"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_Amax);

        Slider_Amax = new QwtSlider(widget_2);
        Slider_Amax->setObjectName(QString::fromUtf8("Slider_Amax"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, Slider_Amax);


        verticalLayout_8->addLayout(formLayout_2);


        verticalLayout_10->addWidget(widget_2);

        widget_3 = new QWidget(frame);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        verticalLayout_9 = new QVBoxLayout(widget_3);
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setContentsMargins(11, 11, 11, 11);
        verticalLayout_9->setObjectName(QString::fromUtf8("verticalLayout_9"));
        label_3 = new QLabel(widget_3);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        verticalLayout_9->addWidget(label_3);

        formLayout_3 = new QFormLayout();
        formLayout_3->setSpacing(6);
        formLayout_3->setObjectName(QString::fromUtf8("formLayout_3"));
        doubleSpinBox_Vmax = new QDoubleSpinBox(widget_3);
        doubleSpinBox_Vmax->setObjectName(QString::fromUtf8("doubleSpinBox_Vmax"));

        formLayout_3->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_Vmax);

        Slider_Vmax = new QwtSlider(widget_3);
        Slider_Vmax->setObjectName(QString::fromUtf8("Slider_Vmax"));

        formLayout_3->setWidget(0, QFormLayout::FieldRole, Slider_Vmax);


        verticalLayout_9->addLayout(formLayout_3);


        verticalLayout_10->addWidget(widget_3);


        verticalLayout_12->addWidget(frame);

        frame_2 = new QFrame(planner);
        frame_2->setObjectName(QString::fromUtf8("frame_2"));
        frame_2->setFrameShape(QFrame::Panel);
        frame_2->setFrameShadow(QFrame::Sunken);
        verticalLayout_11 = new QVBoxLayout(frame_2);
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setContentsMargins(11, 11, 11, 11);
        verticalLayout_11->setObjectName(QString::fromUtf8("verticalLayout_11"));
        label_5 = new QLabel(frame_2);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setFont(font);

        verticalLayout_11->addWidget(label_5);

        verticalLayout_13 = new QVBoxLayout();
        verticalLayout_13->setSpacing(6);
        verticalLayout_13->setObjectName(QString::fromUtf8("verticalLayout_13"));
        label_6 = new QLabel(frame_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        verticalLayout_13->addWidget(label_6);

        formLayout_4 = new QFormLayout();
        formLayout_4->setSpacing(6);
        formLayout_4->setObjectName(QString::fromUtf8("formLayout_4"));
        Slider_Interval = new QwtSlider(frame_2);
        Slider_Interval->setObjectName(QString::fromUtf8("Slider_Interval"));

        formLayout_4->setWidget(0, QFormLayout::FieldRole, Slider_Interval);

        doubleSpinBox = new QDoubleSpinBox(frame_2);
        doubleSpinBox->setObjectName(QString::fromUtf8("doubleSpinBox"));

        formLayout_4->setWidget(0, QFormLayout::LabelRole, doubleSpinBox);


        verticalLayout_13->addLayout(formLayout_4);


        verticalLayout_11->addLayout(verticalLayout_13);

        verticalLayout_14 = new QVBoxLayout();
        verticalLayout_14->setSpacing(6);
        verticalLayout_14->setObjectName(QString::fromUtf8("verticalLayout_14"));
        label_7 = new QLabel(frame_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        verticalLayout_14->addWidget(label_7);

        formLayout_6 = new QFormLayout();
        formLayout_6->setSpacing(6);
        formLayout_6->setObjectName(QString::fromUtf8("formLayout_6"));
        doubleSpinBox_SamplingTime = new QDoubleSpinBox(frame_2);
        doubleSpinBox_SamplingTime->setObjectName(QString::fromUtf8("doubleSpinBox_SamplingTime"));

        formLayout_6->setWidget(0, QFormLayout::LabelRole, doubleSpinBox_SamplingTime);

        Slider_SamplingTime = new QwtSlider(frame_2);
        Slider_SamplingTime->setObjectName(QString::fromUtf8("Slider_SamplingTime"));

        formLayout_6->setWidget(0, QFormLayout::FieldRole, Slider_SamplingTime);


        verticalLayout_14->addLayout(formLayout_6);


        verticalLayout_11->addLayout(verticalLayout_14);


        verticalLayout_12->addWidget(frame_2);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_12->addItem(verticalSpacer);

        pushButton = new QPushButton(planner);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));

        verticalLayout_12->addWidget(pushButton);

        tabWidget->addTab(planner, QString());
        Traj = new QWidget();
        Traj->setObjectName(QString::fromUtf8("Traj"));
        verticalLayout_4 = new QVBoxLayout(Traj);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QString::fromUtf8("verticalLayout_4"));
        qwtPlot_TrajJerk = new QwtPlot(Traj);
        qwtPlot_TrajJerk->setObjectName(QString::fromUtf8("qwtPlot_TrajJerk"));
        qwtPlot_TrajJerk->setFrameShape(QFrame::NoFrame);
        qwtPlot_TrajJerk->setFrameShadow(QFrame::Plain);

        verticalLayout_4->addWidget(qwtPlot_TrajJerk);

        qwtPlot_TrajAcc = new QwtPlot(Traj);
        qwtPlot_TrajAcc->setObjectName(QString::fromUtf8("qwtPlot_TrajAcc"));

        verticalLayout_4->addWidget(qwtPlot_TrajAcc);

        qwtPlot_TrajVel = new QwtPlot(Traj);
        qwtPlot_TrajVel->setObjectName(QString::fromUtf8("qwtPlot_TrajVel"));

        verticalLayout_4->addWidget(qwtPlot_TrajVel);

        tabWidget->addTab(Traj, QString());
        axisX = new QWidget();
        axisX->setObjectName(QString::fromUtf8("axisX"));
        verticalLayout_3 = new QVBoxLayout(axisX);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        qwtPlot_JerkXapprox = new QwtPlot(axisX);
        qwtPlot_JerkXapprox->setObjectName(QString::fromUtf8("qwtPlot_JerkXapprox"));

        verticalLayout_3->addWidget(qwtPlot_JerkXapprox);

        qwtPlot_AccXapprox = new QwtPlot(axisX);
        qwtPlot_AccXapprox->setObjectName(QString::fromUtf8("qwtPlot_AccXapprox"));

        verticalLayout_3->addWidget(qwtPlot_AccXapprox);

        qwtPlot_VelXapprox = new QwtPlot(axisX);
        qwtPlot_VelXapprox->setObjectName(QString::fromUtf8("qwtPlot_VelXapprox"));

        verticalLayout_3->addWidget(qwtPlot_VelXapprox);

        tabWidget->addTab(axisX, QString());
        axisY = new QWidget();
        axisY->setObjectName(QString::fromUtf8("axisY"));
        verticalLayout_5 = new QVBoxLayout(axisY);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QString::fromUtf8("verticalLayout_5"));
        qwtPlot_JerkYapprox = new QwtPlot(axisY);
        qwtPlot_JerkYapprox->setObjectName(QString::fromUtf8("qwtPlot_JerkYapprox"));

        verticalLayout_5->addWidget(qwtPlot_JerkYapprox);

        qwtPlot_AccYapprox = new QwtPlot(axisY);
        qwtPlot_AccYapprox->setObjectName(QString::fromUtf8("qwtPlot_AccYapprox"));

        verticalLayout_5->addWidget(qwtPlot_AccYapprox);

        qwtPlot_VelYapprox = new QwtPlot(axisY);
        qwtPlot_VelYapprox->setObjectName(QString::fromUtf8("qwtPlot_VelYapprox"));

        verticalLayout_5->addWidget(qwtPlot_VelYapprox);

        tabWidget->addTab(axisY, QString());
        axisZ = new QWidget();
        axisZ->setObjectName(QString::fromUtf8("axisZ"));
        verticalLayout_6 = new QVBoxLayout(axisZ);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QString::fromUtf8("verticalLayout_6"));
        qwtPlot_JerkZapprox = new QwtPlot(axisZ);
        qwtPlot_JerkZapprox->setObjectName(QString::fromUtf8("qwtPlot_JerkZapprox"));

        verticalLayout_6->addWidget(qwtPlot_JerkZapprox);

        qwtPlot_AccZapprox = new QwtPlot(axisZ);
        qwtPlot_AccZapprox->setObjectName(QString::fromUtf8("qwtPlot_AccZapprox"));

        verticalLayout_6->addWidget(qwtPlot_AccZapprox);

        qwtPlot_VelZapprox = new QwtPlot(axisZ);
        qwtPlot_VelZapprox->setObjectName(QString::fromUtf8("qwtPlot_VelZapprox"));

        verticalLayout_6->addWidget(qwtPlot_VelZapprox);

        tabWidget->addTab(axisZ, QString());
        tabIdealX = new QWidget();
        tabIdealX->setObjectName(QString::fromUtf8("tabIdealX"));
        verticalLayout = new QVBoxLayout(tabIdealX);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        qwtPlot_AccXideal = new QwtPlot(tabIdealX);
        qwtPlot_AccXideal->setObjectName(QString::fromUtf8("qwtPlot_AccXideal"));

        verticalLayout->addWidget(qwtPlot_AccXideal);

        qwtPlot_VelXideal = new QwtPlot(tabIdealX);
        qwtPlot_VelXideal->setObjectName(QString::fromUtf8("qwtPlot_VelXideal"));

        verticalLayout->addWidget(qwtPlot_VelXideal);

        qwtPlot_PosXideal = new QwtPlot(tabIdealX);
        qwtPlot_PosXideal->setObjectName(QString::fromUtf8("qwtPlot_PosXideal"));

        verticalLayout->addWidget(qwtPlot_PosXideal);

        tabWidget->addTab(tabIdealX, QString());
        tabIdealY = new QWidget();
        tabIdealY->setObjectName(QString::fromUtf8("tabIdealY"));
        verticalLayout_15 = new QVBoxLayout(tabIdealY);
        verticalLayout_15->setSpacing(6);
        verticalLayout_15->setContentsMargins(11, 11, 11, 11);
        verticalLayout_15->setObjectName(QString::fromUtf8("verticalLayout_15"));
        qwtPlot_AccYideal = new QwtPlot(tabIdealY);
        qwtPlot_AccYideal->setObjectName(QString::fromUtf8("qwtPlot_AccYideal"));
        qwtPlot_AccYideal->setMinimumSize(QSize(0, 0));
        qwtPlot_AccYideal->setMaximumSize(QSize(16777215, 16777214));

        verticalLayout_15->addWidget(qwtPlot_AccYideal);

        qwtPlot_VelYideal = new QwtPlot(tabIdealY);
        qwtPlot_VelYideal->setObjectName(QString::fromUtf8("qwtPlot_VelYideal"));

        verticalLayout_15->addWidget(qwtPlot_VelYideal);

        qwtPlot_PosYideal = new QwtPlot(tabIdealY);
        qwtPlot_PosYideal->setObjectName(QString::fromUtf8("qwtPlot_PosYideal"));

        verticalLayout_15->addWidget(qwtPlot_PosYideal);

        tabWidget->addTab(tabIdealY, QString());
        tabError = new QWidget();
        tabError->setObjectName(QString::fromUtf8("tabError"));
        verticalLayout_2 = new QVBoxLayout(tabError);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        qwtPlot_haussdorff1 = new QwtPlot(tabError);
        qwtPlot_haussdorff1->setObjectName(QString::fromUtf8("qwtPlot_haussdorff1"));

        verticalLayout_2->addWidget(qwtPlot_haussdorff1);

        qwtPlot_haussdorff2 = new QwtPlot(tabError);
        qwtPlot_haussdorff2->setObjectName(QString::fromUtf8("qwtPlot_haussdorff2"));

        verticalLayout_2->addWidget(qwtPlot_haussdorff2);

        qwtPlot_errortraj = new QwtPlot(tabError);
        qwtPlot_errortraj->setObjectName(QString::fromUtf8("qwtPlot_errortraj"));

        verticalLayout_2->addWidget(qwtPlot_errortraj);

        tabWidget->addTab(tabError, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName(QString::fromUtf8("tab_2"));
        tabWidget->addTab(tab_2, QString());
        splitter->addWidget(tabWidget);
        viewer = new Viewer(splitter);
        viewer->setObjectName(QString::fromUtf8("viewer"));
        QSizePolicy sizePolicy6(QSizePolicy::Minimum, QSizePolicy::Preferred);
        sizePolicy6.setHorizontalStretch(20);
        sizePolicy6.setVerticalStretch(0);
        sizePolicy6.setHeightForWidth(viewer->sizePolicy().hasHeightForWidth());
        viewer->setSizePolicy(sizePolicy6);
        splitter->addWidget(viewer);

        horizontalLayout->addWidget(splitter);

        tabWidget_2->addTab(tabTrajectoryApproximation, QString());

        verticalLayout_17->addWidget(tabWidget_2);

        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 979, 24));
        menu_File = new QMenu(menuBar);
        menu_File->setObjectName(QString::fromUtf8("menu_File"));
        menu_Edit = new QMenu(menuBar);
        menu_Edit->setObjectName(QString::fromUtf8("menu_Edit"));
        menu_Window = new QMenu(menuBar);
        menu_Window->setObjectName(QString::fromUtf8("menu_Window"));
        menu_Help = new QMenu(menuBar);
        menu_Help->setObjectName(QString::fromUtf8("menu_Help"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);
        toolBar = new QToolBar(MainWindow);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar);
        toolBar_2 = new QToolBar(MainWindow);
        toolBar_2->setObjectName(QString::fromUtf8("toolBar_2"));
        MainWindow->addToolBar(Qt::TopToolBarArea, toolBar_2);
        QWidget::setTabOrder(tabWidget, pushButton);
        QWidget::setTabOrder(pushButton, doubleSpinBox_Amax);
        QWidget::setTabOrder(doubleSpinBox_Amax, doubleSpinBox_Jmax);
        QWidget::setTabOrder(doubleSpinBox_Jmax, doubleSpinBox_Vmax);
        QWidget::setTabOrder(doubleSpinBox_Vmax, doubleSpinBox_SamplingTime);
        QWidget::setTabOrder(doubleSpinBox_SamplingTime, Slider_Vmax);
        QWidget::setTabOrder(Slider_Vmax, Slider_SamplingTime);
        QWidget::setTabOrder(Slider_SamplingTime, Slider_Interval);
        QWidget::setTabOrder(Slider_Interval, Slider_Amax);
        QWidget::setTabOrder(Slider_Amax, Slider_Jmax);
        QWidget::setTabOrder(Slider_Jmax, doubleSpinBox);

        menuBar->addAction(menu_File->menuAction());
        menuBar->addAction(menu_Edit->menuAction());
        menuBar->addAction(menu_Window->menuAction());
        menuBar->addAction(menu_Help->menuAction());
        menu_File->addAction(action_Open);
        menu_File->addAction(action_Close_2);
        menu_File->addAction(action_Close);
        menu_Window->addAction(actionFull_screen);
        menu_Help->addAction(actionHelp);
        menu_Help->addAction(actionAbout_Soft_Motion_Planner);

        retranslateUi(MainWindow);
        QObject::connect(action_Close, SIGNAL(triggered()), MainWindow, SLOT(close()));
        QObject::connect(Slider_Jmax, SIGNAL(sliderMoved(double)), doubleSpinBox_Jmax, SLOT(setValue(double)));
        QObject::connect(Slider_SamplingTime, SIGNAL(sliderMoved(double)), doubleSpinBox_SamplingTime, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox, SIGNAL(valueChanged(double)), Slider_Interval, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_Amax, SIGNAL(valueChanged(double)), Slider_Amax, SLOT(setValue(double)));
        QObject::connect(Slider_Amax, SIGNAL(sliderMoved(double)), doubleSpinBox_Amax, SLOT(setValue(double)));
        QObject::connect(Slider_Vmax, SIGNAL(sliderMoved(double)), doubleSpinBox_Vmax, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_Jmax, SIGNAL(valueChanged(double)), Slider_Jmax, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_SamplingTime, SIGNAL(valueChanged(double)), Slider_SamplingTime, SLOT(setValue(double)));
        QObject::connect(Slider_Interval, SIGNAL(sliderMoved(double)), doubleSpinBox, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_Vmax, SIGNAL(valueChanged(double)), Slider_Vmax, SLOT(setValue(double)));
        QObject::connect(pushButton, SIGNAL(clicked()), MainWindow, SLOT(computeTraj()));
        QObject::connect(doubleSpinBox_Jmax_3, SIGNAL(valueChanged(double)), Slider_Jmax_3, SLOT(setValue(double)));
        QObject::connect(Slider_Jmax_3, SIGNAL(valueChanged(double)), doubleSpinBox_Jmax_3, SLOT(setValue(double)));
        QObject::connect(Slider_Amax_3, SIGNAL(valueChanged(double)), doubleSpinBox_Amax_3, SLOT(setValue(double)));
        QObject::connect(Slider_Vmax_3, SIGNAL(valueChanged(double)), doubleSpinBox_Vmax_3, SLOT(setValue(double)));
        QObject::connect(Slider_Vmax_3, SIGNAL(valueChanged(double)), Slider_Vmax_3, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_Amax_3, SIGNAL(valueChanged(double)), Slider_Amax_3, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_Vmax_3, SIGNAL(valueChanged(double)), Slider_Vmax_3, SLOT(setValue(double)));
        QObject::connect(Slider_A0, SIGNAL(valueChanged(double)), Slider_A0, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_V0, SIGNAL(valueChanged(double)), Slider_V0, SLOT(setValue(double)));
        QObject::connect(Slider_X0, SIGNAL(valueChanged(double)), Slider_X0, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_A0, SIGNAL(valueChanged(double)), doubleSpinBox_A0, SLOT(setValue(double)));
        QObject::connect(Slider_V0, SIGNAL(valueChanged(double)), doubleSpinBox_V0, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_X0, SIGNAL(valueChanged(double)), doubleSpinBox_X0, SLOT(setValue(double)));
        QObject::connect(Slider_Af, SIGNAL(valueChanged(double)), Slider_Af, SLOT(setValue(double)));
        QObject::connect(Slider_Vf, SIGNAL(valueChanged(double)), Slider_Vf, SLOT(setValue(double)));
        QObject::connect(Slider_Xf, SIGNAL(valueChanged(double)), Slider_Xf, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_Af, SIGNAL(valueChanged(double)), doubleSpinBox_Af, SLOT(setValue(double)));
        QObject::connect(Slider_Vf, SIGNAL(valueChanged(double)), doubleSpinBox_Vf, SLOT(setValue(double)));
        QObject::connect(Slider_Xf, SIGNAL(valueChanged(double)), doubleSpinBox_Xf, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_Af, SIGNAL(valueChanged(double)), Slider_Af, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_Vf, SIGNAL(valueChanged(double)), Slider_Vf, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_Xf, SIGNAL(valueChanged(double)), Slider_Xf, SLOT(setValue(double)));
        QObject::connect(Slider_Af, SIGNAL(valueChanged(double)), doubleSpinBox_Af, SLOT(setValue(double)));
        QObject::connect(Slider_A0, SIGNAL(valueChanged(double)), doubleSpinBox_A0, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_A0, SIGNAL(valueChanged(double)), Slider_A0, SLOT(setValue(double)));
        QObject::connect(Slider_X0, SIGNAL(valueChanged(double)), doubleSpinBox_X0, SLOT(setValue(double)));
        QObject::connect(doubleSpinBox_X0, SIGNAL(valueChanged(double)), Slider_X0, SLOT(setValue(double)));

        tabWidget_2->setCurrentIndex(0);
        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", 0, QApplication::UnicodeUTF8));
        action_Open->setText(QApplication::translate("MainWindow", "&Open file", 0, QApplication::UnicodeUTF8));
        action_Open->setShortcut(QApplication::translate("MainWindow", "Ctrl+O", 0, QApplication::UnicodeUTF8));
        action_Close->setText(QApplication::translate("MainWindow", "E&xit", 0, QApplication::UnicodeUTF8));
        action_Close->setShortcut(QApplication::translate("MainWindow", "Ctrl+Q", 0, QApplication::UnicodeUTF8));
        actionAbout_Soft_Motion_Planner->setText(QApplication::translate("MainWindow", "About &Soft Motion Planner", 0, QApplication::UnicodeUTF8));
        actionFull_screen->setText(QApplication::translate("MainWindow", "Full Screen", 0, QApplication::UnicodeUTF8));
        actionFull_screen->setShortcut(QApplication::translate("MainWindow", "F11", 0, QApplication::UnicodeUTF8));
        actionHelp->setText(QApplication::translate("MainWindow", "Help", 0, QApplication::UnicodeUTF8));
        actionHelp->setShortcut(QApplication::translate("MainWindow", "F1", 0, QApplication::UnicodeUTF8));
        action_Close_2->setText(QApplication::translate("MainWindow", "&Close", 0, QApplication::UnicodeUTF8));
        label_29->setText(QString());
        label_28->setText(QApplication::translate("MainWindow", "Motion", 0, QApplication::UnicodeUTF8));
        label_8->setText(QApplication::translate("MainWindow", "Critical Length", 0, QApplication::UnicodeUTF8));
        label_9->setText(QApplication::translate("MainWindow", "T1", 0, QApplication::UnicodeUTF8));
        label_10->setText(QApplication::translate("MainWindow", "T2", 0, QApplication::UnicodeUTF8));
        label_11->setText(QApplication::translate("MainWindow", "T3", 0, QApplication::UnicodeUTF8));
        label_12->setText(QApplication::translate("MainWindow", "T4", 0, QApplication::UnicodeUTF8));
        label_13->setText(QApplication::translate("MainWindow", "T5", 0, QApplication::UnicodeUTF8));
        label_14->setText(QApplication::translate("MainWindow", "T6", 0, QApplication::UnicodeUTF8));
        label_15->setText(QApplication::translate("MainWindow", "T7", 0, QApplication::UnicodeUTF8));
        label_30->setText(QApplication::translate("MainWindow", "Total time", 0, QApplication::UnicodeUTF8));
        label_20->setText(QApplication::translate("MainWindow", "Kinematic Constraints", 0, QApplication::UnicodeUTF8));
        label_21->setText(QApplication::translate("MainWindow", "Jmax", 0, QApplication::UnicodeUTF8));
        label_22->setText(QApplication::translate("MainWindow", "Amax", 0, QApplication::UnicodeUTF8));
        label_23->setText(QApplication::translate("MainWindow", "Vmax", 0, QApplication::UnicodeUTF8));
        label_24->setText(QApplication::translate("MainWindow", "Initial Conditions", 0, QApplication::UnicodeUTF8));
        label_25->setText(QApplication::translate("MainWindow", "A0", 0, QApplication::UnicodeUTF8));
        label_26->setText(QApplication::translate("MainWindow", "V0", 0, QApplication::UnicodeUTF8));
        label_27->setText(QApplication::translate("MainWindow", "X0", 0, QApplication::UnicodeUTF8));
        label_16->setText(QApplication::translate("MainWindow", "Final Conditions", 0, QApplication::UnicodeUTF8));
        label_17->setText(QApplication::translate("MainWindow", "Af", 0, QApplication::UnicodeUTF8));
        label_18->setText(QApplication::translate("MainWindow", "Vf", 0, QApplication::UnicodeUTF8));
        label_19->setText(QApplication::translate("MainWindow", "Xf", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tabSoftMotionPlanner), QApplication::translate("MainWindow", "Soft Motion Planner", 0, QApplication::UnicodeUTF8));
        label_4->setText(QApplication::translate("MainWindow", "Trajectory Motion Law", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Maximum Jerk", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Maximum Acceleration", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Maximum Velocity", 0, QApplication::UnicodeUTF8));
        label_5->setText(QApplication::translate("MainWindow", "Trajectory Parameters", 0, QApplication::UnicodeUTF8));
        label_6->setText(QApplication::translate("MainWindow", "Number of 3-segment Intervals", 0, QApplication::UnicodeUTF8));
        label_7->setText(QApplication::translate("MainWindow", "Sampling Time", 0, QApplication::UnicodeUTF8));
        pushButton->setText(QApplication::translate("MainWindow", "Compute", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(planner), QApplication::translate("MainWindow", "Planner", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(Traj), QApplication::translate("MainWindow", "Trajectory", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(axisX), QApplication::translate("MainWindow", "Axis X Approx", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(axisY), QApplication::translate("MainWindow", "Axis Y Approx", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(axisZ), QApplication::translate("MainWindow", "Axis Z Approx", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tabIdealX), QApplication::translate("MainWindow", "Ideal X", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tabIdealY), QApplication::translate("MainWindow", "IdealY", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tabError), QApplication::translate("MainWindow", "Error", 0, QApplication::UnicodeUTF8));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QApplication::translate("MainWindow", "Page", 0, QApplication::UnicodeUTF8));
        tabWidget_2->setTabText(tabWidget_2->indexOf(tabTrajectoryApproximation), QApplication::translate("MainWindow", "Trajectory Approximation", 0, QApplication::UnicodeUTF8));
        menu_File->setTitle(QApplication::translate("MainWindow", "&File", 0, QApplication::UnicodeUTF8));
        menu_Edit->setTitle(QApplication::translate("MainWindow", "&Edit", 0, QApplication::UnicodeUTF8));
        menu_Window->setTitle(QApplication::translate("MainWindow", "&Window", 0, QApplication::UnicodeUTF8));
        menu_Help->setTitle(QApplication::translate("MainWindow", "&Help", 0, QApplication::UnicodeUTF8));
        toolBar->setWindowTitle(QApplication::translate("MainWindow", "toolBar", 0, QApplication::UnicodeUTF8));
        toolBar_2->setWindowTitle(QApplication::translate("MainWindow", "toolBar_2", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
