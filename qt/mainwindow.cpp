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
#include <qwt_plot_zoomer.h>
#include <qwt_legend.h>
#include <qwt_legend_item.h>
#include <qwt_plot_marker.h>
#include <qwt_painter.h>
#include <qwt_symbol.h>

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
  //connect(this->pushButton, SIGNAL(clicked(bool)), this, SLOT(computeTraj()) ) ;

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
    this->doubleSpinBox_Vmax_3->setRange(0.0001,2);
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


    _nbCurve = 0;
    _fileName = "";
    _isFullScreen = false;
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

/**
 * 
 * @param str 
 */
void MainWindow::loadSvgFile(string str)
{
  Curve curv;

  this->viewer->curve.push_back(curv);
}

void MainWindow::initializeApproxVariables()
{

  this->viewer->curve.clear();
  this->viewer->updateGL();

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


    MainWindow::initializeApproxVariables();	
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
    // cout << " size stack " << viewer->curve.back().path.size()  << " origin " << curv.path.size() << endl;
    viewer->camera()->setPosition(qglviewer::Vec( 0., 0., 0.5));
    // cout << "nbkey " << curv.nbKeyFrames() << endl;
    viewer->updateGL();
    
    plotMotionLaw(curv);
    plotIdealProfile(curv);
    
    
  }
}


void MainWindow::fullScreen()
{
  if(this->_isFullScreen == false){
    this->showFullScreen();
    this->_isFullScreen = true;
  } 
  else {
    this->_isFullScreen = false;
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
  int nbIntervals_local = 0;
  int segment = 1;
  int segment_local = 1;
  int kk = 0;
  int flag = 0;
  int segment_temp_divis = 1;
  int kkk = 0;
  int hh = 0;
  int count_cond = 0;
  int nbIntervals_global = 0;
  double tic = 0.0;
  double pas_inter = 0.03;
  double longeur_path = 0.0;
  double sum_time = 0.0;
  double haus_sup1 = 0.0;
  double haus_sup2 = 0.0;
  double val_err_max = 0.0;
  double val_err_max_traj = 0.0;
  double err_max_def = 0.001;
  double err_max_chaq_seg = 0.0;

  Curve curv2;
  Curve curv_temp;
  Curve curv_divis;
  Curve curv_temp_divis;

  std::vector<SM_COND_DIM> IC;
  std::vector<SM_COND_DIM> FC;
  std::vector<double> Timp;
  std::vector<int> IntervIndex_global;
  std::vector<int> IntervIndex;
  std::vector<SM_OUTPUT> motion;
  std::vector<double> error;
  std::vector<double> error_traj;
  std::vector<double> dis_a_tracer1;
  std::vector<double> dis_a_tracer2;

  std::list<SubTraj>::iterator iter_temp;

  for(unsigned int i=0 ; i< viewer->curve.size(); i++) {
    viewer->curve[i].setIsDraw(false);
  }

  lim.maxJerk = this->doubleSpinBox_Jmax->value();
  lim.maxAcc = this->doubleSpinBox_Amax->value();
  lim.maxVel = this->doubleSpinBox_Vmax->value();
  tic = this->doubleSpinBox_SamplingTime->value();

  Path_Length(viewer->curve.front().path, &longeur_path);
  nbIntervals_global = int(ceil(longeur_path/pas_inter));

  constructTrajSvg(viewer->curve.front().path,  tic, lim, viewer->curve.front().traj);

  saveTraj("QtIdealTraj2.dat", viewer->curve.begin()->traj);

  viewer->curve.begin()->createPath("QtIdealTraj2.dat");
  viewer->curve.begin()->setIsDraw(true);
  viewer->updateGL();

  curv2.traj.clear();
  IC.resize(nbIntervals_global);
  FC.resize(nbIntervals_global);
  Timp.resize(nbIntervals_global);
  IntervIndex_global.resize(nbIntervals_global + 1);

  if(sm_ComputeCondition(viewer->curve.front().traj, viewer->curve.front().discPoint, IC, FC, Timp, IntervIndex_global)!=0){
    printf("MainWindow::computeTraj() ERROR in sm_ComputeCondition()\n");
    return;
  }

  curv_temp.trajList.resize(nbIntervals_global);

  for(iter_temp=curv_temp.trajList.begin(); iter_temp != curv_temp.trajList.end(); iter_temp++){
    if (iter_temp==curv_temp.trajList.begin()){
      iter_temp->traj.resize(IntervIndex_global[segment] - IntervIndex_global[segment-1] +1);
    }
    else iter_temp->traj.resize(IntervIndex_global[segment] - IntervIndex_global[segment-1]);
    segment ++;
  }

  segment = 1; 
  kk = 0;
  for(iter_temp=curv_temp.trajList.begin(); iter_temp != curv_temp.trajList.end(); iter_temp++){
    for (unsigned int k=0; k < iter_temp->traj.size(); k++){
      iter_temp->traj[k].t = viewer->curve.front().traj[kk].t;
    
      iter_temp->traj[k].Pos[0] = viewer->curve.front().traj[kk].Pos[0];
      iter_temp->traj[k].Pos[1] = viewer->curve.front().traj[kk].Pos[1];
      iter_temp->traj[k].Pos[2] = viewer->curve.front().traj[kk].Pos[2];
    
      iter_temp->traj[k].Vel[0] = viewer->curve.front().traj[kk].Vel[0];
      iter_temp->traj[k].Vel[1] = viewer->curve.front().traj[kk].Vel[1];
      iter_temp->traj[k].Vel[2] = viewer->curve.front().traj[kk].Vel[2];
    
      iter_temp->traj[k].Acc[0] = viewer->curve.front().traj[kk].Acc[0];
      iter_temp->traj[k].Acc[1] = viewer->curve.front().traj[kk].Acc[1];
      iter_temp->traj[k].Acc[2] = viewer->curve.front().traj[kk].Acc[2];
    
      iter_temp->traj[k].u = viewer->curve.front().traj[kk].u;
      iter_temp->traj[k].du = viewer->curve.front().traj[kk].du;
      iter_temp->traj[k].ddu = viewer->curve.front().traj[kk].ddu;
      iter_temp->traj[k].AccNorm = viewer->curve.front().traj[kk].AccNorm;
    
      kk ++;
    }
  }

  for(iter_temp=curv_temp.trajList.begin(); iter_temp != curv_temp.trajList.end(); iter_temp++) {
    flag = 0;
    nbIntervals_local = 1;
    error.clear();
    IC.clear();
    FC.clear();
    val_err_max = 0.0;
    SM_CURVE_DATA curv_donne;
    std::vector<double> Temp_alias(1);
       
    do{
      if(flag == 1){
	nbIntervals_local = nbIntervals_local * 2;
	error.clear();
	IC.clear();
	FC.clear();
	IntervIndex.clear();
	val_err_max = 0.0;
      }

      IC.resize(nbIntervals_local);
      FC.resize(nbIntervals_local);
      Timp.resize(nbIntervals_local);
      IntervIndex.resize(nbIntervals_local + 1);
      segment_local = 1;
      segment_temp_divis = 1;
      kkk = 0;
      std::vector<SM_COND_DIM> IC_seg(1);
      std::vector<SM_COND_DIM> FC_seg(1);
      hh = 0;
      err_max_chaq_seg = 0.0;

      if (sm_ComputeCondition(iter_temp->traj, viewer->curve.front().discPoint, IC, FC, Timp, IntervIndex) != 0){
	printf("Compute Problem \n");
	return;
      }

      curv_divis.trajList.resize(nbIntervals_local);
      std::list<SubTraj>::iterator iter_divis;
      curv_temp_divis.trajList.resize(nbIntervals_local);
      std::list<SubTraj>::iterator iter_temp_divis;
      iter_temp_divis=curv_temp_divis.trajList.begin();

      if (flag == 0) {
	iter_temp_divis->traj.resize(iter_temp->traj.size());
      }
      else{
	for(iter_temp_divis=curv_temp_divis.trajList.begin(); iter_temp_divis != curv_temp_divis.trajList.end(); iter_temp_divis++){
	  if (iter_temp_divis==curv_temp_divis.trajList.begin()){
	    iter_temp_divis->traj.resize(IntervIndex[segment_local] - IntervIndex[segment_local-1] +1);
	  }
	  else iter_temp_divis->traj.resize(IntervIndex[segment_local] - IntervIndex[segment_local-1] );
	  segment_local++;
	}
      }

      for(iter_temp_divis=curv_temp_divis.trajList.begin(); iter_temp_divis != curv_temp_divis.trajList.end(); iter_temp_divis++){
	for (unsigned int k=0; k< iter_temp_divis->traj.size(); k++) {
	  iter_temp_divis->traj[k].t = iter_temp->traj[kkk].t;
	  
	  iter_temp_divis->traj[k].Pos[0] = iter_temp->traj[kkk].Pos[0];
	  iter_temp_divis->traj[k].Pos[1] = iter_temp->traj[kkk].Pos[1];
	  iter_temp_divis->traj[k].Pos[2] = iter_temp->traj[kkk].Pos[2];
        
	  iter_temp_divis->traj[k].Vel[0] = iter_temp->traj[kkk].Vel[0];
	  iter_temp_divis->traj[k].Vel[1] = iter_temp->traj[kkk].Vel[1];
	  iter_temp_divis->traj[k].Vel[2] = iter_temp->traj[kkk].Vel[2];
	  
	  iter_temp_divis->traj[k].Acc[0] = iter_temp->traj[kkk].Acc[0];
	  iter_temp_divis->traj[k].Acc[1] = iter_temp->traj[kkk].Acc[1];
	  iter_temp_divis->traj[k].Acc[2] = iter_temp->traj[kkk].Acc[2];
	  
	  iter_temp_divis->traj[k].u = iter_temp->traj[kkk].u;
	  iter_temp_divis->traj[k].du = iter_temp->traj[kkk].du;
	  iter_temp_divis->traj[k].ddu = iter_temp->traj[kkk].ddu;
	  iter_temp_divis->traj[k].AccNorm = iter_temp->traj[kkk].AccNorm;
	  
	  kkk ++;
	}
      }
      
      count_cond = 0;
      iter_temp_divis = curv_temp_divis.trajList.begin();
      for (iter_divis = curv_divis.trajList.begin(); iter_divis != curv_divis.trajList.end(); iter_divis++){
	int toto;
	error.clear();
	memcpy(IC_seg[0].Axis, IC[hh].Axis, sizeof(SM_COND_DIM));
	memcpy(FC_seg[0].Axis, FC[hh].Axis, sizeof(SM_COND_DIM));
	iter_divis->motion_par_seg.resize(3);
	Temp_alias.clear();
	Temp_alias.push_back(iter_temp_divis->traj.size()*tic);
	sm_SolveWithoutOpt(IC_seg, FC_seg, Temp_alias, iter_divis->motion_par_seg);
	iter_divis->traj.clear();
	convertMotionToCurve(iter_divis->motion_par_seg, tic, 1, iter_divis->traj);
	if(iter_temp_divis->traj.size() <= iter_divis->traj.size()) {toto = iter_temp_divis->traj.size();}
	else {toto = iter_divis->traj.size();}
	Calcul_Error_list(iter_temp_divis->traj, iter_divis->traj, &viewer->curve.front().errorMax, error, &val_err_max, toto);
	if (val_err_max > err_max_chaq_seg){
	  err_max_chaq_seg = val_err_max;
	}
	count_cond ++;
	iter_temp_divis ++;
	hh ++;
      }
      
      if (err_max_chaq_seg < err_max_def){
	for (iter_divis = curv_divis.trajList.begin(); iter_divis != curv_divis.trajList.end(); iter_divis++){
	  for (unsigned int i = 0; i< iter_divis->traj.size(); i++){
	    iter_divis->traj[i].t = sum_time;
	    curv_donne.t = iter_divis->traj[i].t;
	    
	    curv_donne.Pos[0] = iter_divis->traj[i].Pos[0];
	    curv_donne.Pos[1] = iter_divis->traj[i].Pos[1];
	    curv_donne.Pos[2] = iter_divis->traj[i].Pos[2];
	    
	    curv_donne.Vel[0] = iter_divis->traj[i].Vel[0];
	    curv_donne.Vel[1] = iter_divis->traj[i].Vel[1];
	    curv_donne.Vel[2] = iter_divis->traj[i].Vel[2];
		    
	    curv_donne.Acc[0] = iter_divis->traj[i].Acc[0];
	    curv_donne.Acc[1] = iter_divis->traj[i].Acc[1];
	    curv_donne.Acc[2] = iter_divis->traj[i].Acc[2];

	    curv_donne.Jerk[0] = iter_divis->traj[i].Jerk[0];
	    curv_donne.Jerk[1] = iter_divis->traj[i].Jerk[1];
	    curv_donne.Jerk[2] = iter_divis->traj[i].Jerk[2];
    
	    curv2.traj.push_back(curv_donne);
	    sum_time = sum_time + tic;
	  }
	}
      }

      flag = 1;
      cout << "err_max_dans_chaq_segment :  " << err_max_chaq_seg << " " << err_max_def << endl; 
    }while (err_max_chaq_seg > err_max_def);

  }

  plotResults(curv2); 
  Calcul_Error(viewer->curve.begin()->traj, curv2.traj, &viewer->curve.begin()->errorMax, error_traj, &val_err_max_traj);
  plotErrors(curv2, error_traj, &val_err_max_traj);

  cout<<"viewer->curve.begin()->traj.size() = "<<viewer->curve.begin()->traj.size()<<endl;
  cout<<"curv2.traj.size() = "<<curv2.traj.size()<<endl;
  Hausdorff(viewer->curve.begin()->traj, curv2.traj, dis_a_tracer1, dis_a_tracer2, &haus_sup1, &haus_sup2);
  plotHaus(dis_a_tracer1, dis_a_tracer2, &haus_sup1, &haus_sup2);

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
  plotGraph(qwtPlot_TrajJerk, xData_a, yData_a, curv.traj.size(),  (char *)"time (s)", (char *)"Accel (m/s^2)", (char *)"");
  plotGraph(qwtPlot_TrajAcc, xData_v, yData_v, curv.traj.size(),   (char *)"time (s)", (char *)"Velocity (m/s)", (char *)"");
  plotGraph(qwtPlot_TrajVel, xData_ab, yData_ab, curv.traj.size(), (char *)"time (s)", (char *)"absicce (m)", (char *)"");

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

  for (int i  = 0; i < (int)curv.traj.size();i++){
    yData_px_tra [i] = pos_discr_X[i];
    xData_px_tra [i] = curv.traj[i].t;
  }
  for (int i  = 0; i < (int)curv.traj.size();i++){
    yData_vx_tra [i] = vel_discr_X[i];
    //       yData_vx_tra [i] = sqrt(curv.traj[i].Vel[0] * curv.traj[i].Vel[0] + curv.traj[i].Vel[1] * curv.traj[i].Vel[1]);
    //       yData_vx_tra [i] = curv.traj[i].Vel[0];
    xData_vx_tra [i] = curv.traj[i].t;
  }
  for (int i  = 0; i < (int)curv.traj.size();i++){
    yData_ax_tra [i] = acc_discr_X[i];
    //       yData_ax_tra [i] = curv.traj[i].AccNorm;
    xData_ax_tra [i] = curv.traj[i].t;
  }

  plotGraph(qwtPlot_PosXideal, xData_px_tra, yData_px_tra, curv.traj.size(), (char *)"time (s)", (char *)"Pos (m)", (char *)""); 
  plotGraph(qwtPlot_VelXideal, xData_vx_tra, yData_vx_tra, curv.traj.size(), (char *)"time (s)", (char *)"Velocity of traj (m/s)", (char *)""); 
  plotGraph(qwtPlot_AccXideal, xData_ax_tra, yData_ax_tra, curv.traj.size(), (char *)"time (s)", (char *)"Accel of traj (m/s^2)", (char *)""); 

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
    xData_py_tra [i] = curv.traj[i].t;
  }
  for (unsigned int i  = 0; i < curv.traj.size();i++){
    yData_vy_tra [i] = vel_discr_Y[i];
    xData_vy_tra [i] = curv.traj[i].t;
  }
  for (unsigned int i  = 0; i < curv.traj.size();i++){
    yData_ay_tra [i] = acc_discr_Y[i];
    xData_ay_tra [i] = curv.traj[i].t;
  }

  plotGraph(qwtPlot_PosYideal, xData_py_tra, yData_py_tra, curv.traj.size(), (char *)"time (s)", (char *)"Pos (m)", (char *)""); 
  plotGraph(qwtPlot_VelYideal, xData_vy_tra, yData_vy_tra, curv.traj.size(), (char *)"time (s)", (char *)"Velocity of traj (m/s)", (char *)""); 
  plotGraph(qwtPlot_AccYideal, xData_ay_tra, yData_ay_tra, curv.traj.size(), (char *)"time (s)", (char *)"Accel of traj (m/s^2)", (char *)""); 

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

//     plotGraph(qwtPlot_haussdorff1, xData_haus1, yData_haus1, dis_a_tracer1.size(), "sample_point", "dis_hausdorff1", "");
void MainWindow::plotGraph(QwtPlot *p, double xData[], double yData[], int size, char* xName, char *yName, char *title) {
  QwtPlotCurve *curve_ay_tra = new QwtPlotCurve("plot");
  QwtPlotZoomer *zoom = NULL;
  curve_ay_tra->setData(xData,yData, size);
  curve_ay_tra->attach(p);
  p->setAxisTitle(p->xBottom, xName );
  p->setAxisTitle(p->yLeft, yName );
  QwtPlotGrid *g = new QwtPlotGrid;
  g->enableXMin(true);
  g->enableYMin(true);
  g->setMajPen(QPen(Qt::black, 0, Qt::DotLine));
  g->setMinPen(QPen(Qt::gray, 0 , Qt::DotLine));
  g->attach(p);
  zoom = new QwtPlotZoomer(p->xBottom, p->yLeft, p->canvas());
  p->replot();
  p->setCanvasBackground(QColor(Qt::white));
  return;

}



void MainWindow::plotErrors(Curve &curv2, std::vector<double> &error, double *val_err_max) {
  double *xData_error = NULL, *yData_error  = NULL;
  xData_error  = (double* ) malloc (sizeof(double) * curv2.traj.size());
  yData_error  = (double* ) malloc (sizeof(double) * curv2.traj.size());
  for (unsigned int i  = 0; i < curv2.traj.size();i++){
    xData_error [i] = curv2.traj[i].t;
    yData_error [i] = error[i];
  }
  plotGraph(qwtPlot_errortraj, xData_error, yData_error, curv2.traj.size(), (char *)"time (s)", (char *)"error (m)", (char *)"");
  QwtPlotCurve *error_approx = new QwtPlotCurve("error_approx");
  error_approx->setData(xData_error,yData_error, curv2.traj.size());
  error_approx->attach(qwtPlot_errortraj);
  QString label_error_approx;
  label_error_approx.sprintf("err_app: %.3g (m)", *val_err_max);
  qwtPlot_errortraj->setAxisTitle(qwtPlot_errortraj->xBottom, label_error_approx );
  qwtPlot_errortraj->setAxisTitle(qwtPlot_errortraj->yLeft, "error_approx" );

  free(xData_error);
  free(yData_error);

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

  plotGraph(qwtPlot_JerkXapprox, xData_J_X, yData_J_X, curv2.traj.size(), (char *)"time (s)", (char *)"Jerk (m/s^3)", (char *)"");
  plotGraph(qwtPlot_JerkYapprox, xData_J_Y, yData_J_Y, curv2.traj.size(), (char *)"time (s)", (char *)"Jerk (m/s^3)", (char *)"");
  plotGraph(qwtPlot_JerkZapprox, xData_J_Z, yData_J_Z, curv2.traj.size(), (char *)"time (s)", (char *)"Jerk (m/s^3)", (char *)"");

  plotGraph(qwtPlot_AccXapprox, xData_A_X, yData_A_X, curv2.traj.size(), (char *)"time (s)", (char *)"Accelaration (m/s^2)", (char *)"");
  plotGraph(qwtPlot_AccYapprox, xData_A_Y, yData_A_Y, curv2.traj.size(), (char *)"time (s)", (char *)"Accelaration (m/s^2)", (char *)"");
  plotGraph(qwtPlot_AccZapprox, xData_A_Z, yData_A_Z, curv2.traj.size(), (char *)"time (s)", (char *)"Accelaration (m/s^2)", (char *)"");
									   		       
  plotGraph(qwtPlot_VelXapprox, xData_V_X, yData_V_X, curv2.traj.size(), (char *)"time (s)", (char *)"Velocity (m/s)", (char *)"");
  plotGraph(qwtPlot_VelYapprox, xData_V_Y, yData_V_Y, curv2.traj.size(), (char *)"time (s)", (char *)"Velocity (m/s)", (char *)"");
  plotGraph(qwtPlot_VelZapprox, xData_V_Z, yData_V_Z, curv2.traj.size(), (char *)"time (s)", (char *)"Velocity (m/s)", (char *)"");

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
    
  //cout << "plotResults_succes"<<endl;
  return;
}

void MainWindow::plotHaus(std::vector<double> &dis_a_tracer1, std::vector<double> &dis_a_tracer2, double *sup1, double *sup2){
  double *xData_haus1 = NULL, *yData_haus1  = NULL;
  double *xData_haus2 = NULL, *yData_haus2  = NULL;
  QwtPlotZoomer *zoom1 = NULL, *zoom2 = NULL;
  xData_haus1  = (double* ) malloc (sizeof(double) * dis_a_tracer1.size());
  yData_haus1  = (double* ) malloc (sizeof(double) * dis_a_tracer1.size());
  xData_haus2  = (double* ) malloc (sizeof(double) * dis_a_tracer2.size());
  yData_haus2  = (double* ) malloc (sizeof(double) * dis_a_tracer2.size());
  for (unsigned int i  = 0; i < dis_a_tracer1.size();i++){
    xData_haus1 [i] = i;
    yData_haus1 [i] = dis_a_tracer1[i];
  }
  for (unsigned int i  = 0; i < dis_a_tracer2.size();i++){
    xData_haus2 [i] = i;
    yData_haus2 [i] = dis_a_tracer2[i];
  }
  //     plotGraph(qwtPlot_haussdorff1, xData_haus1, yData_haus1, dis_a_tracer1.size(), "sample_point", "dis_hausdorff1", "");
  //     plotGraph(qwtPlot_haussdorff2, xData_haus2, yData_haus2, dis_a_tracer2.size(), "sample_point", "dis_hausdorff2", "");

  QwtPlotCurve *haus1 = new QwtPlotCurve("haus1");
  haus1->setData(xData_haus1,yData_haus1, dis_a_tracer1.size());
  haus1->attach(qwtPlot_haussdorff1);
  QString label_haus_peak1;
  label_haus_peak1.sprintf("dis_hau_1: %.3g (m)", *sup1);
  qwtPlot_haussdorff1->setAxisTitle(qwtPlot_haussdorff1->xBottom, label_haus_peak1 );
  qwtPlot_haussdorff1->setAxisTitle(qwtPlot_haussdorff1->yLeft, "dis_hausdorff1" );
  QwtPlotGrid *g1 = new QwtPlotGrid;
  g1->enableXMin(true);
  g1->enableYMin(true);
  g1->setMajPen(QPen(Qt::black, 0, Qt::DotLine));
  g1->setMinPen(QPen(Qt::gray, 0 , Qt::DotLine));
  g1->attach(qwtPlot_haussdorff1);
  zoom1 = new QwtPlotZoomer(qwtPlot_haussdorff1->xBottom, qwtPlot_haussdorff1->yLeft, qwtPlot_haussdorff1->canvas());
  qwtPlot_haussdorff1->replot();
  qwtPlot_haussdorff1->setCanvasBackground(QColor(Qt::white));


  QwtPlotCurve *haus2 = new QwtPlotCurve("haus2");
  haus2->setData(xData_haus2,yData_haus2, dis_a_tracer2.size());
  haus2->attach(qwtPlot_haussdorff2);
  QString label_haus_peak2;
  label_haus_peak2.sprintf("dis_hau_2: %.3g (m)", *sup2);
  qwtPlot_haussdorff2->setAxisTitle(qwtPlot_haussdorff2->xBottom, label_haus_peak2 );
  qwtPlot_haussdorff2->setAxisTitle(qwtPlot_haussdorff2->yLeft, "dis_hausdorff2" );
  QwtPlotGrid *g2 = new QwtPlotGrid;
  g2->enableXMin(true);
  g2->enableYMin(true);
  g2->setMajPen(QPen(Qt::black, 0, Qt::DotLine));
  g2->setMinPen(QPen(Qt::gray, 0 , Qt::DotLine));
  g2->attach(qwtPlot_haussdorff2);
  zoom2 = new QwtPlotZoomer(qwtPlot_haussdorff2->xBottom, qwtPlot_haussdorff2->yLeft, qwtPlot_haussdorff2->canvas());
  qwtPlot_haussdorff2->replot();
  qwtPlot_haussdorff2->setCanvasBackground(QColor(Qt::white)); 

  free(xData_haus1);
  free(yData_haus1);
  free(xData_haus2);
  free(yData_haus2);

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
    

  /* Sliders of Kinematic cobnstraints */
  lim.maxJerk = this->Slider_Jmax_3->value();
  lim.maxAcc  = this->Slider_Amax_3->value();
  lim.maxVel  = this->Slider_Vmax_3->value();

  this->Slider_A0->setRange(-lim.maxAcc, lim.maxAcc,0.0001);
  this->Slider_V0->setRange(-lim.maxVel,lim.maxVel,0.0001);
  this->Slider_Af->setRange(-lim.maxAcc, lim.maxAcc,0.0001);
  this->Slider_Vf->setRange(-lim.maxVel,lim.maxVel,0.0001);

  /* Sliders of Initial conditions*/
  ICloc.a = this->Slider_A0->value();
  ICloc.v = this->Slider_V0->value();
  ICloc.x = this->Slider_X0->value();

  /* Sliders of Final conditions*/
  FCloc.a = this->Slider_Af->value();
  FCloc.v = this->Slider_Vf->value();
  FCloc.x = this->Slider_Xf->value();

  if(sm_ComputeSoftMotionLocal(ICloc, FCloc, lim, &TimeSeg, &TrajectoryType, &dcOut, &zoneOut) == 0) {

    //traiter l erruer
  }

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

  /* Conpute background curve */
  ICloc.a = 0.0;
  ICloc.v = -lim.maxVel;
  ICloc.x = 0.0;

  FCloc.a = 0.0;
  FCloc.v = -lim.maxVel;
  FCloc.x = 0.2;

  lim.maxJerk = this->Slider_Jmax_3->value();
  lim.maxAcc = this->Slider_Amax_3->value();
  lim.maxVel = this->Slider_Vmax_3->value();

  

  if(sm_ComputeSoftMotionLocal(ICloc, FCloc, lim, &TimeSegFond, &TrajectoryTypeFond, &dcOutFond, &zoneOutFond)!=0){
    this->qwtPlot_2->clear();
    this->qwtPlot->clear();
    this->qwtPlot_2->replot();
    this->qwtPlot->replot();
    printf("**** ERROR softMotion-libs ****\n");
    return;
  }

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

  this->qwtPlot_2->clear();
  this->qwtPlot->clear();
  this->qwtPlot->setCanvasBackground(QColor(Qt::white));
  this->qwtPlot_2->setCanvasBackground(QColor(Qt::white));

  QwtPlotCurve *curve_pos = new QwtPlotCurve("Pos--(m)");
  QPen pen_pos = curve_pos->pen();
  pen_pos.setColor(Qt::darkYellow);
  pen_pos.setWidth(2);
  pen_pos.setStyle(Qt::DashDotLine);
  curve_pos->setPen(pen_pos);
  curve_pos->setData(t,u, nbPoints);
  curve_pos->attach(this->qwtPlot_2);

  QwtPlotCurve *curve_vel = new QwtPlotCurve("Vel--(m/s)");
  QPen pen_vel = curve_vel->pen();
  pen_vel.setColor(Qt::darkGreen);
  pen_vel.setWidth(2);
  curve_vel->setPen(pen_vel);
  curve_vel->setData(t,du, nbPoints);
  curve_vel->attach(this->qwtPlot_2);

  QwtPlotCurve *curve_acc = new QwtPlotCurve("Acc--(m/s^2)");
  QPen pen_acc = curve_acc->pen();
  pen_acc.setColor(Qt::blue);
  pen_acc.setWidth(2);
  curve_acc->setPen(pen_acc);
  curve_acc->setData(t,ddu, nbPoints);
  curve_acc->attach(this->qwtPlot_2);

  QwtLegend *legend_droite = new QwtLegend;
  qwtPlot_2->insertLegend(legend_droite, QwtPlot::BottomLegend);

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

  QwtPlotCurve *curve_velacc = new QwtPlotCurve("Vel_Acc");
  QPen pen_velacc = curve_velacc->pen();
  pen_velacc.setColor(Qt::red);
  pen_velacc.setWidth(2);
  curve_velacc->setPen(pen_velacc);
  curve_velacc->setData(du,ddu, nbPoints);
  curve_velacc->attach(this->qwtPlot);

  QwtPlotCurve *curve_velaccFond = new QwtPlotCurve("Vel_Acc_Lim");
  QPen pen_velaccFond = curve_velaccFond->pen();
  pen_velaccFond.setColor(Qt::blue);
  pen_velaccFond.setWidth(1);
  curve_velaccFond->setPen(pen_velaccFond);
  curve_velaccFond->setData(duFond,dduFond, nbPointsFond);
  curve_velaccFond->attach(this->qwtPlot);

  /*legend*/
  QwtLegend *legend_gauche = new QwtLegend;
  qwtPlot->insertLegend(legend_gauche, QwtPlot::BottomLegend);

  QwtPlotGrid *g2 = new QwtPlotGrid;
  g2->enableXMin(true);
  g2->enableYMin(true);
  this->qwtPlot->setAxisScale(qwtPlot->yLeft,-lim.maxAcc-(lim.maxAcc/10.0) , lim.maxAcc+(lim.maxAcc/10.0),0.1);
  this->qwtPlot->setAxisScale(qwtPlot->xBottom,-lim.maxVel-(lim.maxVel/10.0) , lim.maxVel+(lim.maxVel/10.0),0.1);
  g2->setMajPen(QPen(Qt::black, 0, Qt::DotLine));
  g2->setMinPen(QPen(Qt::gray, 0 , Qt::DotLine));
  g2->attach(this->qwtPlot);

  /*marker*/
  QwtPlotMarker *d_mrk1 = new QwtPlotMarker();
  d_mrk1->setSymbol( QwtSymbol(QwtSymbol::Diamond, QColor(Qt::green), QColor(Qt::green), QSize(10,10)));
  d_mrk1->attach(this->qwtPlot);
  d_mrk1->setValue(du[0], ddu[0]);
  d_mrk1->setLabelAlignment(Qt::AlignLeft | Qt::AlignTop);
  QString label1;
  label1.sprintf("IC");
  QwtText text1(label1);
  d_mrk1->setLabel(text1);

  QwtPlotMarker *d_mrk2 = new QwtPlotMarker();
  d_mrk2->setSymbol( QwtSymbol(QwtSymbol::Diamond, QColor(Qt::red), QColor(Qt::red), QSize(10,10)));
  d_mrk2->attach(this->qwtPlot);
  d_mrk2->setValue(du[nbPoints-1], ddu[nbPoints-1]);
  d_mrk2->setLabelAlignment(Qt::AlignRight | Qt::AlignBottom);
  QString label2;
  label2.sprintf("FC");
  QwtText text2(label2);
  d_mrk2->setLabel(text2);



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


/*
  void MainWindow::computeTraj()
  {
  SM_LIMITS lim;
  double tic;
  int nbIntervals;
  Curve curv2;

  double haus_sup1 = 0.0;
  double haus_sup2 = 0.0;
  double val_err_max = 0.0;



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
  //curv2.traj.resize(viewer->curve.begin()->traj.size());
    
  if (sm_SolveWithoutOpt(IC, FC, Timp, motion) != 0){
  printf("Solve Problem \n");
  return;
  }
  curv2.traj.clear();
  convertMotionToCurve(motion, tic, nbIntervals, curv2.traj);
  plotResults(curv2);

  Calcul_Error(viewer->curve.begin()->traj, curv2.traj, &viewer->curve.begin()->errorMax, error, &val_err_max);
  plotErrors(curv2, error, &val_err_max);

  Hausdorff(viewer->curve.begin()->traj, curv2.traj, dis_a_tracer1, dis_a_tracer2, &haus_sup1, &haus_sup2);
  plotHaus(dis_a_tracer1, dis_a_tracer2, &haus_sup1, &haus_sup2);

  saveTraj("QtApproxTraj.dat", curv2.traj);
  curv2.createPath("QtApproxTraj.dat");
  curv2.setIsDraw(true);
  viewer->curve.push_back(curv2);

  viewer->updateGL();
  return;
  }
*/
