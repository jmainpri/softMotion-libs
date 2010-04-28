#include "mainwindow.h"

#include "ui_mainwindow.h"

#include <string>
#include <iostream>


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
#include "QGLViewer/qglviewer.h"
#include "qwt_plot.h"
#include "qwt_slider.h"
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_legend.h>

 #include <QFileDialog>

 #include "../src/softMotion.h"
using namespace std;

// the definition of the fonction MainWindow
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    Ui_MainWindow()
{
    this->setupUi(this);

    setWindowTitle(QApplication::translate("MainWindow", "Soft Motion Planner", 0, QApplication::UnicodeUTF8));

    this->Slider_Jmax->setRange(0,60,0.001);
    this->doubleSpinBox_Jmax->setSingleStep(0.1);
    this->doubleSpinBox_Jmax->setRange(0,60);
    this->doubleSpinBox_Jmax->setDecimals(4);

    this->Slider_Amax->setRange(0,5,0.01);
    this->doubleSpinBox_Amax->setSingleStep(0.01);
    this->doubleSpinBox_Amax->setRange(0,5);
    this->doubleSpinBox_Amax->setDecimals(4);

    this->Slider_Vmax->setRange(0,2,0.001);
    this->doubleSpinBox_Vmax->setRange(0,2);
    this->doubleSpinBox_Vmax->setSingleStep(0.001);
    this->doubleSpinBox_Vmax->setDecimals(4);

    this->Slider_SamplingTime->setRange(0.0001,0.01,0.0001);
    this->doubleSpinBox_SamplingTime->setRange(0.0001,0.01);
    this->doubleSpinBox_SamplingTime->setSingleStep(0.001);	
    this->doubleSpinBox_SamplingTime->setDecimals(4);

    this->doubleSpinBox_Jmax->setValue(0.9);
    this->doubleSpinBox_Amax->setValue(0.3);
    this->doubleSpinBox_Vmax->setValue(0.02);
    this->doubleSpinBox_SamplingTime->setValue(0.001);
    this->Slider_Interval->setValue(100);
    
    this->doubleSpinBox_SamplingTime->setValue(0.001);

/*number of interval here*/
    this->doubleSpinBox->setRange(0,500);
    this->doubleSpinBox->setDecimals(0);
    this->doubleSpinBox->setSingleStep(10);
    this->doubleSpinBox->setValue(100);

    
    connect(this->action_Open,SIGNAL(triggered()),this,SLOT(openFile()));
    connect(this->actionFull_screen, SIGNAL(triggered()),this,SLOT(fullScreen()));
    connect(this->action_Close_2, SIGNAL(triggered(bool)), this, SLOT(closeFile()));
    connect(this->pushButton, SIGNAL(clicked(bool)), this, SLOT(computeTraj()) ) ;



    ////////////////////////////////////////////////////////////
    //////  SoftMotion Planner                    /////////////
    ///////////////////////////////////////////////////////////
    
    this->Slider_Jmax_3->setRange(0,60,0.001);
    this->doubleSpinBox_Jmax_3->setSingleStep(0.1);
    this->doubleSpinBox_Jmax_3->setRange(0,60);
    this->doubleSpinBox_Jmax_3->setDecimals(4);
    this->Slider_Jmax_3->setValue(0.9);
    this->doubleSpinBox_Jmax_3->setValue(0.9);

    this->Slider_Amax_3->setRange(0,5,0.001);
    this->doubleSpinBox_Amax_3->setSingleStep(0.01);
    this->doubleSpinBox_Amax_3->setRange(0,5);
    this->doubleSpinBox_Amax_3->setDecimals(4);
    this->Slider_Amax_3->setValue(0.3);
    this->doubleSpinBox_Amax_3->setValue(0.3);

    this->Slider_Vmax_3->setRange(0,2,0.001);
    this->doubleSpinBox_Vmax_3->setSingleStep(0.01);
    this->doubleSpinBox_Vmax_3->setRange(0,2);
    this->doubleSpinBox_Vmax_3->setDecimals(4);
    this->Slider_Vmax_3->setValue(0.3);
    this->doubleSpinBox_Vmax_3->setValue(0.15);


    this->Slider_A0->setRange(-5,5,0.001);
    this->doubleSpinBox_A0->setSingleStep(0.01);
    this->doubleSpinBox_A0->setRange(-5,5);
    this->doubleSpinBox_A0->setDecimals(4);
    this->Slider_A0->setValue(0.0);
    this->doubleSpinBox_A0->setValue(0.0);

    this->Slider_V0->setRange(-2,2,0.001);
    this->doubleSpinBox_V0->setSingleStep(0.01);
    this->doubleSpinBox_V0->setRange(-2,2);
    this->doubleSpinBox_V0->setDecimals(4);
    this->Slider_V0->setValue(0.0);
    this->doubleSpinBox_V0->setValue(0.0);

    this->Slider_Af->setRange(-5,5,0.001);
    this->doubleSpinBox_Af->setSingleStep(0.01);
    this->doubleSpinBox_Af->setRange(-5,5);
    this->doubleSpinBox_Af->setDecimals(4);
    this->Slider_Af->setValue(0.0);
    this->doubleSpinBox_Af->setValue(0.0);

    this->Slider_Vf->setRange(-2,2,0.001);
    this->doubleSpinBox_Vf->setSingleStep(0.01);
    this->doubleSpinBox_Vf->setRange(-5,5);
    this->doubleSpinBox_Vf->setDecimals(4);
    this->Slider_Vf->setValue(0.0);
    this->doubleSpinBox_Vf->setValue(0.0);

    this->Slider_Xf->setRange(-0.3,0.3,0.001);
    this->doubleSpinBox_Xf->setSingleStep(0.001);
    this->doubleSpinBox_Xf->setRange(-0.3,0.3);
    this->doubleSpinBox_Xf->setDecimals(4);
    this->Slider_Xf->setValue(0.1);
    this->doubleSpinBox_Xf->setValue(0.1);


    
    connect(this->Slider_Jmax_3, SIGNAL(valueChanged(double)), this, SLOT(computeSoftMotion()));
    connect(this->Slider_Amax_3, SIGNAL(valueChanged(double)), this, SLOT(computeSoftMotion()));
    connect(this->Slider_Vmax_3, SIGNAL(valueChanged(double)), this, SLOT(computeSoftMotion()));
    
    connect(this->Slider_Af, SIGNAL(valueChanged(double)), this, SLOT(computeSoftMotion()));
    connect(this->Slider_Vf, SIGNAL(valueChanged(double)), this, SLOT(computeSoftMotion()));
    connect(this->Slider_Xf, SIGNAL(valueChanged(double)), this, SLOT(computeSoftMotion()));
    
    connect(this->Slider_A0, SIGNAL(valueChanged(double)), this, SLOT(computeSoftMotion()));
    connect(this->Slider_V0, SIGNAL(valueChanged(double)), this, SLOT(computeSoftMotion()));
    // connect(this->Slider_X0, SIGNAL(valueChanged(double)), this, SLOT(computeSoftMotion()));

}

MainWindow::~MainWindow()
{

}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type()) {
    case QEvent::LanguageChange:
        this->retranslateUi(this);
        break;
    default:
        break;
    }
}

void MainWindow::loadSvgFile(string str)
{
    Curve curv;

    this->viewer->curve.push_back(curv);
}

void MainWindow::openFile()
{
    QString fileName = QFileDialog::getOpenFileName(this);
    std::string str;
    std::string str2;
    SM_LIMITS Lim;
    Curve curv;

    if (!fileName.isEmpty()){
	    this->fileName = fileName.toStdString();
        cout << "Open file " << this->fileName << endl;
        str.clear();
        str += "Soft Motion Planner : ";
        str += this->fileName.c_str();

        setWindowTitle(QApplication::translate("MainWindow", str.c_str(), 0, QApplication::UnicodeUTF8));


	if (parseSvg(this->fileName.c_str(), curv.path, &curv.width, &curv.height) == SM_ERROR) {
        cout << " parse ERROR" << endl;
	    return;
	}
        cout << " parse is OK" << endl;

   // Custom Params
    Lim.maxJerk = this->doubleSpinBox_Jmax->value();
    Lim.maxAcc  = this->doubleSpinBox_Amax->value();
    Lim.maxVel  = this->doubleSpinBox_Vmax->value();


	
/* computation of the trajectory*/
	constructTrajSvg(curv.path,this->doubleSpinBox_SamplingTime->value(), Lim, curv.traj);

	str2.clear();
	str2 += "QtIdealTraj.dat";
	saveTraj(str2, curv.traj);
	   
        /* Handle the path */
	curv.createPath(str2.c_str());
	curv.draw();
 	curv.setIsDraw(true);
 	viewer->curve.push_back(curv);
         cout << " size stack " << viewer->curve.back().path.size()  << " origin " << curv.path.size() << endl;
	viewer->camera()->setPosition(qglviewer::Vec( 0., 0., 0.5));
	cout << "nbkey " << curv.nbKeyFrames() << endl;
	viewer->updateGL();

  	plotMotionLaw(curv);
        plotIdealProfile(curv);


    }
}



void MainWindow::fullScreen()
{
    if(this->isFullScreen == false){
        this->showFullScreen();
        this->isFullScreen = true;
    } 
    else {
        this->isFullScreen = false;
        this->showMaximized();
    }
    return;
}

void MainWindow::closeFile()
{
    this->viewer->curve.clear();
    this->viewer->updateGL();
    setWindowTitle(QApplication::translate("MainWindow","Soft Motion Planner", 0, QApplication::UnicodeUTF8));
    return;
}

void MainWindow::computeTraj()
{
    SM_LIMITS lim;
    double tic;
    int nbIntervals;
    Curve curv2;

    for(unsigned int i=0 ; i< viewer->curve.size(); i++) {
        viewer->curve[i].setIsDraw(false);
    }

    std::vector<SM_COND_DIM> IC;
    std::vector<SM_COND_DIM> FC;
    std::vector<double> Timp;
    std::vector<int> IntervIndex;
    std::vector<SM_OUTPUT> motion;
    std::vector<double> error;
    std::vector<double> dis_a_tracer1;
    std::vector<double> dis_a_tracer2;

    lim.maxJerk = this->doubleSpinBox_Jmax->value();
    lim.maxAcc = this->doubleSpinBox_Amax->value();
    lim.maxVel = this->doubleSpinBox_Vmax->value();
    tic = this->doubleSpinBox_SamplingTime->value();

    viewer->curve.begin()->traj.clear();
    cout << "nbcurve " << viewer->curve.size() << endl;
    cout << "nbpath " << viewer->curve.begin()->path.size() << endl;
    cout << "construct " << viewer->curve.begin()->path.back().subpath.size() <<" " <<  viewer->curve.begin()->traj.size() << endl;
    constructTrajSvg(viewer->curve.begin()->path,  tic, lim, viewer->curve.begin()->traj);
    cout << " save " << endl;
    saveTraj("QtIdealTraj2.dat", viewer->curve.begin()->traj);

    viewer->curve.begin()->createPath("QtIdealTraj2.dat");
    viewer->curve.begin()->setIsDraw(true);
    viewer->updateGL();

    nbIntervals = this->Slider_Interval->value();
    IC.resize(nbIntervals);
    FC.resize(nbIntervals);
    Timp.resize(nbIntervals);
    IntervIndex.resize(nbIntervals + 1);

    if (sm_ComputeCondition(viewer->curve.begin()->traj, viewer->curve.begin()->discPoint, IC, FC, Timp, IntervIndex) != 0){
        printf("Compute Problem \n");
        return;
    }

    motion.resize(viewer->curve.begin()->traj.size());
    curv2.traj.resize(viewer->curve.begin()->traj.size());
    
    if (sm_SolveWithoutOpt(IC, FC, Timp, motion) != 0){
        printf("Solve Problem \n");
        return;
    }

    convertMotionToCurve(motion, tic, nbIntervals, curv2.traj);
    plotResults(curv2);

    Calcul_Error(viewer->curve.begin()->traj, curv2.traj, &viewer->curve.begin()->errorMax, error);
    plotErrors(curv2, error);

    Hausdorff(viewer->curve.begin()->traj, curv2.traj, dis_a_tracer1, dis_a_tracer2);
    plotHaus(dis_a_tracer1, dis_a_tracer2);
    saveTraj("QtApproxTraj.dat", curv2.traj);
    curv2.createPath("QtApproxTraj.dat");
    curv2.setIsDraw(true);
    viewer->curve.push_back(curv2);

    viewer->updateGL();
    return;
}

void MainWindow::plotMotionLaw(Curve &curv) {
    //computation of the figures velocity, accelaration, and abscisse
        /* velocity*/
    double *xData_v = NULL, *yData_v  = NULL;
    xData_v = (double* ) malloc (sizeof(double) * curv.traj.size());
    yData_v = (double* ) malloc (sizeof(double) * curv.traj.size());
    for (unsigned int i  = 0; i < curv.traj.size();i++){
        yData_v [i] = curv.traj[i].du;
        xData_v [i] = curv.traj[i].t;
    }
        /* accelaration*/
    double *xData_a = NULL, *yData_a  = NULL;
    xData_a = (double* ) malloc (sizeof(double) * curv.traj.size());
    yData_a = (double* ) malloc (sizeof(double) * curv.traj.size());
    for (unsigned int i  = 0; i < curv.traj.size();i++){
        yData_a[i] = curv.traj[i].ddu;
        xData_a[i] = curv.traj[i].t;
    }
        /* abscicca*/
    double *xData_ab = NULL, *yData_ab  = NULL;
    xData_ab = (double* ) malloc (sizeof(double) * curv.traj.size());
    yData_ab = (double* ) malloc (sizeof(double) * curv.traj.size());
    for (unsigned int i  = 0; i < curv.traj.size();i++){
        yData_ab [i] = curv.traj[i].u;
        xData_ab [i] = curv.traj[i].t;
    }

        /* plot accel, vel and abscicca*/
    plotGraph(qwtPlot_TrajJerk, xData_a, yData_a, curv.traj.size(), "time (s)", "Accel (m/s^2)", "");
    plotGraph(qwtPlot_TrajAcc, xData_v, yData_v, curv.traj.size(), "time (s)", "Velocity (m/s)", "");
    plotGraph(qwtPlot_TrajVel, xData_ab, yData_ab, curv.traj.size(), "time (s)", "absicce (m)", "");

    qwtPlot_TrajJerk->replot();
    qwtPlot_TrajAcc->replot();
    qwtPlot_TrajVel->replot();

    free(xData_a);
    free(yData_a);
    free(xData_v);
    free(yData_v);
    free(xData_ab);
    free(yData_ab);
    return;
}

void MainWindow::plotIdealProfile(Curve &curv) {
    // Vitesse_et_acceleration profile en traj
    std::vector<double> vel_discr_X;
    std::vector<double> vel_discr_Y;
    std::vector<double> acc_discr_X;
    std::vector<double> acc_discr_Y;
    std::vector<double> pos_discr_X;
    std::vector<double> pos_discr_Y;

    Vel_Profile(curv.traj, vel_discr_X, vel_discr_Y, acc_discr_X, acc_discr_Y, pos_discr_X, pos_discr_Y);

    /* Axis X */
    double *xData_px_tra = NULL, *yData_px_tra  = NULL;    
    double *xData_vx_tra = NULL, *yData_vx_tra  = NULL;
    double *xData_ax_tra = NULL, *yData_ax_tra  = NULL;
    xData_px_tra = (double* ) malloc (sizeof(double) * curv.traj.size());
    yData_px_tra = (double* ) malloc (sizeof(double) * curv.traj.size());
    xData_vx_tra = (double* ) malloc (sizeof(double) * curv.traj.size());
    yData_vx_tra = (double* ) malloc (sizeof(double) * curv.traj.size());
    xData_ax_tra = (double* ) malloc (sizeof(double) * curv.traj.size());
    yData_ax_tra = (double* ) malloc (sizeof(double) * curv.traj.size());

    for (int i  = 0; i < curv.traj.size();i++){
      yData_px_tra [i] = pos_discr_X[i];
      xData_px_tra [i] = i;
    }
    for (int i  = 0; i < curv.traj.size();i++){
      yData_vx_tra [i] = vel_discr_X[i];
      xData_vx_tra [i] = i;
    }
    for (int i  = 0; i < curv.traj.size();i++){
      yData_ax_tra [i] = acc_discr_X[i];
      xData_ax_tra [i] = i;
    }

    plotGraph(qwtPlot_PosXideal, xData_px_tra, yData_px_tra, curv.traj.size(), "time (s)", "Pos (m)", ""); 
    plotGraph(qwtPlot_VelXideal, xData_vx_tra, yData_vx_tra, curv.traj.size(), "time (s)", "Velocity of traj (m/s)", ""); 
    plotGraph(qwtPlot_AccXideal, xData_ax_tra, yData_ax_tra, curv.traj.size(), "time (s)", "Accel of traj (m/s^2)", ""); 

    /* Axis Y */
    double *xData_py_tra = NULL, *yData_py_tra  = NULL;    
    double *xData_vy_tra = NULL, *yData_vy_tra  = NULL;
    double *xData_ay_tra = NULL, *yData_ay_tra  = NULL;
    xData_py_tra = (double* ) malloc (sizeof(double) * curv.traj.size());
    yData_py_tra = (double* ) malloc (sizeof(double) * curv.traj.size());
    xData_vy_tra = (double* ) malloc (sizeof(double) * curv.traj.size());
    yData_vy_tra = (double* ) malloc (sizeof(double) * curv.traj.size());
    xData_ay_tra = (double* ) malloc (sizeof(double) * curv.traj.size());
    yData_ay_tra = (double* ) malloc (sizeof(double) * curv.traj.size());

    for (unsigned int i  = 0; i < curv.traj.size();i++){
        yData_py_tra [i] = pos_discr_Y[i];
        xData_py_tra [i] = i;
    }
    for (unsigned int i  = 0; i < curv.traj.size();i++){
        yData_vy_tra [i] = vel_discr_Y[i];
        xData_vy_tra [i] = i;
    }
    for (unsigned int i  = 0; i < curv.traj.size();i++){
        yData_ay_tra [i] = acc_discr_Y[i];
        xData_ay_tra [i] = i;
    }

    plotGraph(qwtPlot_PosYideal, xData_py_tra, yData_py_tra, curv.traj.size(), "time (s)", "Pos (m)", ""); 
    plotGraph(qwtPlot_VelYideal, xData_vy_tra, yData_vy_tra, curv.traj.size(), "time (s)", "Velocity of traj (m/s)", ""); 
    plotGraph(qwtPlot_AccYideal, xData_ay_tra, yData_ay_tra, curv.traj.size(), "time (s)", "Accel of traj (m/s^2)", ""); 

    /* free the memory */
    free(xData_px_tra);
    free(yData_px_tra);
    free(xData_vx_tra);
    free(yData_vx_tra);
    free(xData_ax_tra);
    free(yData_ax_tra);
    free(xData_py_tra);
    free(yData_py_tra);
    free(xData_vy_tra);
    free(yData_vy_tra);
    free(xData_ay_tra);
    free(yData_ay_tra);
    return;
}

void MainWindow::plotGraph(QwtPlot *p, double xData[], double yData[], int size, char* xName, char *yName, char *title) {
    QwtPlotCurve *curve_ay_tra = new QwtPlotCurve("plot");
    curve_ay_tra->setData(xData,yData, size-1);
    curve_ay_tra->attach(p);
    p->setAxisTitle(p->xBottom, xName );
    p->setAxisTitle(p->yLeft, yName );
    QwtPlotGrid *g = new QwtPlotGrid;
    g->enableXMin(true);
    g->enableYMin(true);
    g->setMajPen(QPen(Qt::black, 0, Qt::DotLine));
    g->setMinPen(QPen(Qt::gray, 0 , Qt::DotLine));
    g->attach(p);
    QwtPlotZoomer *zoom = new QwtPlotZoomer(p->xBottom, p->yLeft, p->canvas());
    p->replot();
    p->show();

    return;

}



void MainWindow::plotErrors(Curve &curv2, std::vector<double> &error) {
    double *xData_error = NULL, *yData_error  = NULL;
    xData_error  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    yData_error  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    for (unsigned int i  = 0; i < curv2.traj.size();i++){
        xData_error [i] = curv2.traj[i].t;
        yData_error [i] = error[i];
    }
    plotGraph(qwtPlot_errortraj, xData_error, yData_error, curv2.traj.size(), "time (s)", "error (m)", "");
//     cout << " error max0 " << viewer->curve[0].errorMax.kc[0].x<< " " << viewer->curve[0].errorMax.kc[1].x<< endl;
//     cout << " error max1 " << viewer->curve[1].errorMax.kc[0].x<< " " << viewer->curve[1].errorMax.kc[1].x<< endl;

    free(xData_error);
    free(yData_error);

    cout << "ploterror_succes"<<endl;
    return;
}


void MainWindow::plotResults(Curve &curv2) {
    
    /* computation of the figures velocity, accelaration, and abscicca*/

    /* jerk*/
    double *xData_J_X = NULL, *yData_J_X  = NULL;
    double *xData_J_Y = NULL, *yData_J_Y  = NULL;
    double *xData_J_Z = NULL, *yData_J_Z  = NULL;
    xData_J_X  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    yData_J_X  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    xData_J_Y  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    yData_J_Y  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    xData_J_Z  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    yData_J_Z  = (double* ) malloc (sizeof(double) * curv2.traj.size());

    for (unsigned int i  = 0; i < curv2.traj.size();i++){
        xData_J_X [i] = curv2.traj[i].t;
        yData_J_X [i] = curv2.traj[i].Jerk[0];
    }
    for (unsigned int i  = 0; i < curv2.traj.size();i++){
        xData_J_Y [i] = curv2.traj[i].t;
        yData_J_Y [i] = curv2.traj[i].Jerk[1];
    }
    for (unsigned int i  = 0; i < curv2.traj.size();i++){
        xData_J_Z [i] = curv2.traj[i].t;
        yData_J_Z [i] = curv2.traj[i].Jerk[2];
    }

    /* acceleration*/
    double *xData_A_X = NULL, *yData_A_X  = NULL;
    double *xData_A_Y = NULL, *yData_A_Y  = NULL;
    double *xData_A_Z = NULL, *yData_A_Z  = NULL;
    xData_A_X  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    yData_A_X  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    xData_A_Y  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    yData_A_Y  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    xData_A_Z  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    yData_A_Z  = (double* ) malloc (sizeof(double) * curv2.traj.size());

    for (unsigned int i  = 0; i < curv2.traj.size();i++){
        xData_A_X [i] = curv2.traj[i].t;
        yData_A_X [i] = curv2.traj[i].Acc[0];
    }
    for (unsigned int i  = 0; i < curv2.traj.size();i++){
        xData_A_Y [i] = curv2.traj[i].t;
        yData_A_Y [i] = curv2.traj[i].Acc[1];
    }
    for (unsigned int i  = 0; i < curv2.traj.size();i++){
        xData_A_Z [i] = curv2.traj[i].t;
        yData_A_Z [i] = curv2.traj[i].Acc[2];
    }

    /* velocity*/
    double *xData_V_X = NULL, *yData_V_X  = NULL;
    double *xData_V_Y = NULL, *yData_V_Y  = NULL;
    double *xData_V_Z = NULL, *yData_V_Z  = NULL;
    xData_V_X  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    yData_V_X  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    xData_V_Y  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    yData_V_Y  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    xData_V_Z  = (double* ) malloc (sizeof(double) * curv2.traj.size());
    yData_V_Z  = (double* ) malloc (sizeof(double) * curv2.traj.size());

    for (unsigned int i  = 0; i < curv2.traj.size();i++){
        xData_V_X [i] = curv2.traj[i].t;
        yData_V_X [i] = curv2.traj[i].Vel[0];
    }
    for (unsigned int i  = 0; i < curv2.traj.size();i++){
        xData_V_Y [i] = curv2.traj[i].t;
        yData_V_Y [i] = curv2.traj[i].Vel[1];
    }
    for (unsigned int i  = 0; i < curv2.traj.size();i++){
        xData_V_Z [i] = curv2.traj[i].t;
        yData_V_Z [i] = curv2.traj[i].Vel[2];
    }

    plotGraph(qwtPlot_JerkXapprox, xData_J_X, yData_J_X, curv2.traj.size(), "time (s)", "Jerk (m/s^3)", "");
    plotGraph(qwtPlot_JerkYapprox, xData_J_Y, yData_J_Y, curv2.traj.size(), "time (s)", "Jerk (m/s^3)", "");
    plotGraph(qwtPlot_JerkZapprox, xData_J_Z, yData_J_Z, curv2.traj.size(), "time (s)", "Jerk (m/s^3)", "");

    plotGraph(qwtPlot_AccXapprox, xData_A_X, yData_A_X, curv2.traj.size(), "time (s)", "Accelaration (m/s^2)", "");
    plotGraph(qwtPlot_AccYapprox, xData_A_Y, yData_A_Y, curv2.traj.size(), "time (s)", "Accelaration (m/s^2)", "");
    plotGraph(qwtPlot_AccZapprox, xData_A_Z, yData_A_Z, curv2.traj.size(), "time (s)", "Accelaration (m/s^2)", "");

    plotGraph(qwtPlot_VelXapprox, xData_V_X, yData_V_X, curv2.traj.size(), "time (s)", "Velocity (m/s)", "");
    plotGraph(qwtPlot_VelYapprox, xData_V_Y, yData_V_Y, curv2.traj.size(), "time (s)", "Velocity (m/s)", "");
    plotGraph(qwtPlot_VelZapprox, xData_V_Z, yData_V_Z, curv2.traj.size(), "time (s)", "Velocity (m/s)", "");

    qwtPlot_JerkXapprox->replot();
    qwtPlot_JerkYapprox->replot();
    qwtPlot_JerkZapprox->replot();
    qwtPlot_AccXapprox->replot();
    qwtPlot_AccYapprox->replot();
    qwtPlot_AccZapprox->replot();
    qwtPlot_VelXapprox->replot();
    qwtPlot_VelYapprox->replot();
    qwtPlot_VelZapprox->replot();

    free(xData_J_X);
    free(yData_J_X);
    free(xData_J_Y);
    free(yData_J_Y);
    free(xData_J_Z);
    free(yData_J_Z);

    free(xData_A_X);
    free(yData_A_X);
    free(xData_A_Y);
    free(yData_A_Y);
    free(xData_A_Z);
    free(yData_A_Z);

    free(xData_V_X);
    free(yData_V_X);
    free(xData_V_Y);
    free(yData_V_Y);
    free(xData_V_Z);
    free(yData_V_Z);
    
    cout << "plotResults_succes"<<endl;
    return;
}

void plotHaus(std::vector<double> &dis_a_tracer1, std::vector<double> &dis_a_tracer2){
    int size = dis_a_tracer1.size();
    double *xData_haus1 = NULL, *yData_haus1  = NULL;
    double *xData_haus2 = NULL, *yData_haus2  = NULL;
    xData_haus1  = (double* ) malloc (sizeof(double) * size);
    yData_haus1  = (double* ) malloc (sizeof(double) * size);
    xData_haus2  = (double* ) malloc (sizeof(double) * size);
    yData_haus2  = (double* ) malloc (sizeof(double) * size);

    for (unsigned int i  = 0; i < size;i++){
        xData_haus1 [i] = i;
        yData_haus1 [i] = dis_a_tracer1[i];
    }
    for (unsigned int i  = 0; i < size;i++){
        xData_haus2 [i] = i;
        yData_haus2 [i] = dis_a_tracer2[i];
    }

    plotGraph(qwtPlot_haussdorff1, xData_haus1, yData_haus1, size, "time (s)", "dis_hausdorff", "");
    plotGraph(qwtPlot_haussdorff2, xData_haus2, yData_haus2, size, "time (s)", "dis_hausdorff", "");

    return;
}

void MainWindow::computeSoftMotion()
{

  SM_COND ICloc, FCloc;
  SM_LIMITS lim;
  SM_TIMES TimeSeg, TimeSegFond;
  int TrajectoryType, TrajectoryTypeFond;
  double dcOut, dcOutFond;
  int zoneOut, zoneOutFond, i;
  double Time[7];
  double IC[3];
  double J[7];
  double total_time=0.0;
  int nbPoints=0, nbPointsFond=0;
  double tic = 0.005;
  double tloc = 0.0;
  double *t=NULL, *ddu=NULL, *du=NULL, *u=NULL;
  double *tFond=NULL, *dduFond=NULL, *duFond=NULL, *uFond=NULL;
    


  lim.maxJerk = this->Slider_Jmax_3->value();
  lim.maxAcc  = this->Slider_Amax_3->value();
  lim.maxVel  = this->Slider_Vmax_3->value();

  this->Slider_A0->setRange(-lim.maxAcc, lim.maxAcc,0.0001);
  this->Slider_V0->setRange(-lim.maxVel,lim.maxVel,0.0001);
  this->Slider_Af->setRange(-lim.maxAcc, lim.maxAcc,0.0001);
  this->Slider_Vf->setRange(-lim.maxVel,lim.maxVel,0.0001);


  ICloc.a = this->Slider_A0->value();
  ICloc.v = this->Slider_V0->value();
  ICloc.x = this->Slider_X0->value();

  FCloc.a = this->Slider_Af->value();
  FCloc.v = this->Slider_Vf->value();
  FCloc.x = this->Slider_Xf->value();

  sm_ComputeSoftMotionLocal(ICloc, FCloc, lim, &TimeSeg, &TrajectoryType, &dcOut, &zoneOut);

  this->doubleSpinBox_T1->setValue(TimeSeg.Tjpa);
  this->doubleSpinBox_T2->setValue(TimeSeg.Taca);
  this->doubleSpinBox_T3->setValue(TimeSeg.Tjna);
  this->doubleSpinBox_T4->setValue(TimeSeg.Tvc);
  this->doubleSpinBox_T5->setValue(TimeSeg.Tjnb);
  this->doubleSpinBox_T6->setValue(TimeSeg.Tacb);
  this->doubleSpinBox_T7->setValue(TimeSeg.Tjpb);
  this->doubleSpinBox_dc->setValue(dcOut);


  IC[0] = ICloc.a;
  IC[1] = ICloc.v;
  IC[2] = ICloc.x;

  Time[0] = TimeSeg.Tjpa;
  Time[1] = TimeSeg.Taca;
  Time[2] = TimeSeg.Tjna;
  Time[3] = TimeSeg.Tvc;
  Time[4] = TimeSeg.Tjnb;
  Time[5] = TimeSeg.Tacb;
  Time[6] = TimeSeg.Tjpb;

  J[0] = TrajectoryType*lim.maxJerk;
  J[1] = 0.0;
  J[2] = -TrajectoryType* lim.maxJerk;
  J[3] = 0.0;
  J[4] = -TrajectoryType* lim.maxJerk;
  J[5] = 0.0;
  J[6] = TrajectoryType*lim.maxJerk;

  for (i = 0; i < 7; i++){
    total_time = total_time + Time[i]; // total_time is the time for every 7 segment
  }
  nbPoints = ((int) (total_time/tic)) + 1; // nbPoints is the point discretized by step of 0.001
  
  this->doubleSpinBox_totalTime->setValue(total_time);

  t   = (double *) malloc(sizeof(double) * nbPoints);
  ddu = (double *) malloc(sizeof(double) * nbPoints);
  du  = (double *) malloc(sizeof(double) * nbPoints);
  u   = (double *) malloc(sizeof(double) * nbPoints);

  for (i = 0; i < nbPoints; i++){
    tloc = i * tic;
    if (tloc >= total_time) {
      tloc =total_time;
    }
    t[i] = tloc;
  }

    sm_AVX_TimeVar(IC, Time, J, 7, t, nbPoints, ddu, du, u);

    
  ICloc.a = 0.0;
  ICloc.v = -lim.maxVel;
  ICloc.x = 0.0;

  FCloc.a = 0.0;
  FCloc.v = -lim.maxVel;
  FCloc.x = 0.2;

  lim.maxJerk = this->Slider_Jmax_3->value();
  lim.maxAcc = this->Slider_Amax_3->value();
  lim.maxVel = this->Slider_Vmax_3->value();

  

  sm_ComputeSoftMotionLocal(ICloc, FCloc, lim, &TimeSegFond, &TrajectoryTypeFond, &dcOutFond, &zoneOutFond);

  IC[0] = ICloc.a;
  IC[1] = ICloc.v;
  IC[2] = ICloc.x;

  Time[0] = TimeSegFond.Tjpa;
  Time[1] = TimeSegFond.Taca;
  Time[2] = TimeSegFond.Tjna;
  Time[3] = TimeSegFond.Tvc;
  Time[4] = TimeSegFond.Tjnb;
  Time[5] = TimeSegFond.Tacb;
  Time[6] = TimeSegFond.Tjpb;

  J[0] = lim.maxJerk;
  J[1] = 0.0;
  J[2] = - lim.maxJerk;
  J[3] = 0.0;
  J[4] = - lim.maxJerk;
  J[5] = 0.0;
  J[6] = lim.maxJerk;
  total_time = 0.0;

  for (i = 0; i < 7; i++){
  
    total_time = total_time + Time[i]; // total_time is the time for every 7 segment
  }
  nbPointsFond = ((int) (total_time/tic)) + 1; // nbPoints is the point discretized by step of 0.001

  tFond   = (double *) malloc(sizeof(double) * nbPointsFond);
  dduFond = (double *) malloc(sizeof(double) * nbPointsFond);
  duFond  = (double *) malloc(sizeof(double) * nbPointsFond);
  uFond   = (double *) malloc(sizeof(double) * nbPointsFond);

  for (i = 0; i < nbPointsFond; i++){
    tloc = i * tic;
    if (tloc >= total_time) {
      tloc =total_time;
    }
    tFond[i] = tloc;
  }

    sm_AVX_TimeVar(IC, Time, J, 7, tFond, nbPointsFond, dduFond, duFond, uFond);

    // plot
    this->qwtPlot_2->clear();
    this->qwtPlot->clear();
    this->qwtPlot->setCanvasBackground(QColor(Qt::white));
    this->qwtPlot_2->setCanvasBackground(QColor(Qt::white));
    
    QwtPlotCurve *curve_pos = new QwtPlotCurve("plot");
    QPen pen_pos = curve_pos->pen();
    pen_pos.setColor(Qt::darkYellow);
    pen_pos.setWidth(2);
    curve_pos->setPen(pen_pos);
    curve_pos->setData(t,u, nbPoints);
    curve_pos->attach(this->qwtPlot_2);

    QwtPlotCurve *curve_vel = new QwtPlotCurve("plot");
       QPen pen_vel = curve_vel->pen();
    pen_vel.setColor(Qt::darkGreen);
      pen_vel.setWidth(2);
    curve_vel->setPen(pen_vel);
    curve_vel->setData(t,du, nbPoints);
    curve_vel->attach(this->qwtPlot_2);

    QwtPlotCurve *curve_acc = new QwtPlotCurve("plot");
           QPen pen_acc = curve_acc->pen();
    pen_acc.setColor(Qt::blue);
      pen_acc.setWidth(2);
    curve_acc->setPen(pen_acc);
    curve_acc->setData(t,ddu, nbPoints);
    curve_acc->attach(this->qwtPlot_2);
    //     p->setAxisTitle(p->xBottom, xName );
//     p->setAxisTitle(p->yLeft, yName );
    QwtPlotGrid *g = new QwtPlotGrid;
    g->enableXMin(true);
    g->enableYMin(true);
    g->setMajPen(QPen(Qt::black, 0, Qt::DotLine));
    g->setMinPen(QPen(Qt::gray, 0 , Qt::DotLine));
    g->attach(this->qwtPlot_2);
//     if(lim.maxAcc > lim.maxVel) {
//     this->qwtPlot_2->setAxisScale(qwtPlot_2->yLeft,-lim.maxAcc-(lim.maxAcc/10.0) , lim.maxAcc+(lim.maxAcc/10.0),0.1);
//     } else {
//     this->qwtPlot_2->setAxisScale(qwtPlot_2->yLeft,-lim.maxVel-(lim.maxVel/10.0) , lim.maxVel+(lim.maxVel/10.0),0.1);
//     }
    this->qwtPlot_2->replot();

    QwtPlotCurve *curve_velacc = new QwtPlotCurve("plot");
               QPen pen_velacc = curve_velacc->pen();
    pen_velacc.setColor(Qt::red);
      pen_velacc.setWidth(2);
    curve_velacc->setPen(pen_velacc);
    curve_velacc->setData(du,ddu, nbPoints);
    curve_velacc->attach(this->qwtPlot);

       QwtPlotCurve *curve_velaccFond = new QwtPlotCurve("plot");
      QPen pen_velaccFond = curve_velaccFond->pen();
    pen_velaccFond.setColor(Qt::blue);
      pen_velaccFond.setWidth(1);
    curve_velaccFond->setPen(pen_velaccFond);
     curve_velaccFond->setData(duFond,dduFond, nbPointsFond);
//     curve_velaccFond->setData(tFond,duFond, nbPointsFond);
    curve_velaccFond->attach(this->qwtPlot);

    QwtPlotGrid *g2 = new QwtPlotGrid;
    g2->enableXMin(true);
    g2->enableYMin(true);
    this->qwtPlot->setAxisScale(qwtPlot->yLeft,-lim.maxAcc-(lim.maxAcc/10.0) , lim.maxAcc+(lim.maxAcc/10.0),0.1);
    this->qwtPlot->setAxisScale(qwtPlot->xBottom,-lim.maxVel-(lim.maxVel/10.0) , lim.maxVel+(lim.maxVel/10.0),0.1);
    g2->setMajPen(QPen(Qt::black, 0, Qt::DotLine));
    g2->setMinPen(QPen(Qt::gray, 0 , Qt::DotLine));
    g2->attach(this->qwtPlot);
    this->qwtPlot->replot();

    free(t);
    free(u);
    free(du);
    free(ddu);

    free(tFond);
    free(uFond);
    free(duFond);
    free(dduFond);
    return;
}

