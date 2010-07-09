#include <string>
#include <iostream>
#include <fstream>

#ifdef ENABLE_DISPLAY
#include "ui_mainwindow.h"


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
#include <qprogressbar.h>
#include <QFileDialog>

#endif

#include "time_proto.h"
#include "softMotion.h"

#include "QSoftMotionPlanner.h"

using namespace std;

/** @file QSoftMotionPlanner.cpp
 * @brief This file includes all useful functions for creating an interface of SoftMotion
 */

// the definition of the fonction QSoftMotionPlanner
QSoftMotionPlanner::QSoftMotionPlanner(
#ifdef ENABLE_DISPLAY 
QWidget *parent
#endif
) 
#ifdef ENABLE_DISPLAY
: QMainWindow(parent),
  Ui_MainWindow()
#endif
{
  int type_courbe;


#ifdef ENABLE_DISPLAY
    this->setupUi(this);

    setWindowTitle(QApplication::translate("QSoftMotionPlanner", "Soft Motion Planner", 0, QApplication::UnicodeUTF8));

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

    /*desired error here*/
    this->Slider_desError->setRange(0,0.01, 0.000001); 
    this->doubleSpinBox_DesError->setRange(0,0.01);
    this->doubleSpinBox_DesError->setDecimals(6);
    this->doubleSpinBox_DesError->setSingleStep(0.000001);
    this->doubleSpinBox_DesError->setValue(0.001);

    
    connect(this->action_Open,SIGNAL(triggered()),this,SLOT(openFile()));
    connect(this->actionFull_screen, SIGNAL(triggered()),this,SLOT(fullScreen()));
    connect(this->action_Close_2, SIGNAL(triggered(bool)), this, SLOT(closeFile()));
    //connect(this->pushButton, SIGNAL(clicked(bool)), this, SLOT(computeTraj()) ) ;
  
    connect(this->pushButtonGenFile, SIGNAL(clicked(bool)), this, SLOT(genFileTraj()));

    connect(this->pushButtonComputeHauss, SIGNAL(clicked(bool)), this, SLOT(computeHausdorff()) ) ;

    connect(this->checkBox, SIGNAL(clicked(bool)), this, SLOT(setDraw()) ) ;
    this->checkBox->setChecked(Qt::Checked);   

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


  #else
    double Jmax=0.0, Amax=0.0, Vmax=0.0, sampling =0.001, err = 0.001; 
    int step;

    cout<<endl<<"**** Set the Approx and Motion Law Parameters please ****"<<endl;
	cout << "Jmax : " << endl;
      cin>> Jmax;
	cout << "Amax : " << endl;
      cin>> Amax;
	cout << "Vmax : " << endl;
      cin>> Vmax;
	cout << "sampling : " << endl;
      cin>> sampling;
	cout << "errMax : " << endl;
      cin>> err;
      cout << "time step for the exported file : "<< endl; 
     cin>> step;

      _lim.maxJerk = Jmax;
      _lim.maxAcc  = Amax;
      _lim.maxVel  = Vmax;
      _sampling = sampling;
      _errMax = err;
      _timeStep = step;

    cout<<endl<<"**** Choose a path type please ****"<<endl;
    cout<<"0--none"<<endl<<"1--droite"<<endl<<"2--circle"<<endl<<"3--sinusoid"<<endl<<"4--parabole"<<endl<<"5--file"<<endl;
    cin>>type_courbe;
    switch (type_courbe){
    case 0:                       break;
    case 1:   defineFunction_l(); break;
    case 2:   defineFunction_c(); break;
    case 3:   defineFunction_s(); break;
    case 4:   defineFunction_p(); break;
    case 5:   
      {
	cout << "Absolute file path : " << endl;
	cin>> _fileName;
	cout << "open the file " << _fileName << endl;
	openFile();
      }         
      break;
    default:                      break;
    }
    
  #endif
  _nbCurve = 0;
  _fileName = "";
  _isFullScreen = false;
  _curve.clear();
    
}

QSoftMotionPlanner::~QSoftMotionPlanner()
{
  _curve.clear();

}

void QSoftMotionPlanner::genFileTraj(){
#ifdef ENABLE_DISPLAY
  QApplication::setOverrideCursor(Qt::WaitCursor);
#endif
  int incr = _timeStep;
  FILE *fp = NULL;
  fp = fopen("output.traj", "w");
  if(fp==NULL) {
    std::cerr << " cannont open file to write the trajectory" << std::endl; 
    return;
  }

  cout << "Number of positions in the file " <<_curve.back().traj.size() << endl;
  for (unsigned int i = 0; i < _curve.back().traj.size(); i += incr){
    fprintf(fp, "%lf\t", _curve.back().traj[i].Pos[0]);
    fprintf(fp, "%lf\t", _curve.back().traj[i].Pos[1]);
    fprintf(fp, "%lf\n", _curve.back().traj[i].Pos[2]);
  }
  fclose(fp);
#ifdef ENABLE_DISPLAY
  QApplication::setOverrideCursor(Qt::ArrowCursor);
#endif
  return;
}

void QSoftMotionPlanner::computeHausdorff(){
#ifdef ENABLE_DISPLAY
  QApplication::setOverrideCursor(Qt::WaitCursor);
#endif
  std::vector<double> dis_a_tracer1;
  std::vector<double> dis_a_tracer2;
  double sup1 = 0.0;
  double sup2 = 0.0;
  double haus_sup1 = 0.0;
  double haus_sup2 = 0.0;
  double dis_hausdorff = 0.0;
  double w = 0.0;

  // f1 pour calculer la distance la plus longue entre courbe1 et courbe2
  for (int i=0; i< (int)_curve.front().traj.size(); i++){
    std::vector<double> dis1;
    for (int j=0; j<  (int)_curve.back().traj.size(); j++){
      w = sqrt(pow((_curve.front().traj[i].Pos[0]-_curve.back().traj[j].Pos[0]),2) +  pow((_curve.front().traj[i].Pos[1]-_curve.back().traj[j].Pos[1]),2) + pow((_curve.front().traj[i].Pos[2]-_curve.back().traj[j].Pos[2]),2));
      dis1.push_back (w);
    }
    double inf1 = dis1[0];
    for (int k=0; k<(int)_curve.back().traj.size(); k++){
      if (dis1[k]<inf1) {inf1 = dis1[k];}
    }
    dis_a_tracer1.push_back(inf1);
  }
  sup1 = dis_a_tracer1[0];
  for (int m=0; m<(int)_curve.front().traj.size(); m++){
    if (dis_a_tracer1[m]>(sup1)) {sup1 = dis_a_tracer1[m];}
  }
    
  // f2 pour calculer la distance la plus longue entre courbe2 et courbe1
  for (int i=0; i< (int)_curve.back().traj.size(); i++){
    std::vector<double> dis2;
    for (int j=0; j< (int)_curve.front().traj.size(); j++){
      w = sqrt  (pow((_curve.back().traj[i].Pos[0]-_curve.front().traj[j].Pos[0]),2) + pow((_curve.back().traj[i].Pos[1]-_curve.front().traj[j].Pos[1]),2) + pow((_curve.back().traj[i].Pos[2]-_curve.front().traj[j].Pos[2]),2));
      dis2.push_back(w);
    }
    double inf2 = dis2[0];
    for (int k=0; k<(int)_curve.front().traj.size(); k++){
      if (dis2[k]<inf2) {inf2 = dis2[k];}
    }
    dis_a_tracer2.push_back(inf2);
  }
  sup2 = dis_a_tracer2[0];
  for (int m=0; m<(int)_curve.back().traj.size() ; m++){
    if (dis_a_tracer2[m]>(sup2)) {sup2 = dis_a_tracer2[m];}
  }

  // calcul de la distance hausdorff
  dis_hausdorff = (sup1 > sup2 ? sup1 : sup2);
  cout << "Hausdorff distance = " << dis_hausdorff << endl;
#ifdef ENABLE_DISPLAY
  _plot.plotHaus(dis_a_tracer1, dis_a_tracer2, haus_sup1, haus_sup2, qwtPlot_haussdorff1, qwtPlot_haussdorff2);
  QApplication::setOverrideCursor(Qt::ArrowCursor);
#endif
  return; 
}


void QSoftMotionPlanner::setDraw() 
{
  for(unsigned int i=0 ; i< _curve.size(); i++) {
    _curve[i].setIsDraw(display());
  }
  return;
}

#ifdef ENABLE_DISPLAY
void QSoftMotionPlanner::changeEvent(QEvent *e)
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
#endif

void QSoftMotionPlanner::loadSvgFile(string str)
{
  Curve curv;
  this->_curve.push_back(curv);
}

void QSoftMotionPlanner::initializeApproxVariables()
{
  this->_curve.clear();
#ifdef ENABLE_DISPLAY
  this->viewer->updateGL();
#endif

}

void QSoftMotionPlanner::defineFunction_p(){
  Curve curv;
  SM_LIMITS Lim;
  Path lpath;
  SubPath lsubpath;
  std::string str2;
  double a = 5.0;
  double start_x = 0.0;
  double end_x = 0.02;

  lsubpath.parabol.a = a;
  lsubpath.parabol.start_x = start_x;
  lsubpath.parabol.end_x = end_x;
  lsubpath.type = PARABOL;
  lpath.subpath.push_back(lsubpath);
  curv.path.push_back(lpath);  
  constructTrajSvg(curv.path, _sampling, _lim, curv.traj);
  
  str2.clear();
  str2 += "cercle_traj.dat";
  saveTraj(str2, curv.traj);
  curv.draw();
  curv.setIsDraw(display());
  _curve.push_back(curv);

#ifdef ENABLE_DISPLAY
    viewer->curve.push_back(curv);
  viewer->camera()->setPosition(qglviewer::Vec( 0., 0., 0.5));
  viewer->updateGL();

  _plot.plotMotionLaw(curv, qwtPlot_TrajJerk, qwtPlot_TrajAcc, qwtPlot_TrajVel);
  _plot.plotIdealProfile(curv, qwtPlot_PosXideal, qwtPlot_VelXideal, qwtPlot_AccXideal,
			 qwtPlot_PosYideal, qwtPlot_VelYideal, qwtPlot_AccYideal);
#else 

  computeTraj();
  genFileTraj();

#endif
  return;  
}

/* the definition of a cercle x = acos(2PIft); y = asin(2PIft) */
void QSoftMotionPlanner::defineFunction_c(){
  Curve curv;
  SM_LIMITS Lim;
  Path lpath;
  SubPath lsubpath;
  std::string str2;
  double a1 = 0.1;
  double f1 = 20.0;

  lsubpath.cercle.center.x = 0.0;
  lsubpath.cercle.center.y = 0.0;
  lsubpath.cercle.radius = a1;
  lsubpath.cercle.sinus_para.frequency = f1;
  lsubpath.type = CERCLE;
  lpath.subpath.push_back(lsubpath);
  curv.path.push_back(lpath);  
  constructTrajSvg(curv.path, _sampling, _lim, curv.traj);
  
  str2.clear();
  str2 += "cercle_traj.dat";
  saveTraj(str2, curv.traj);
  curv.draw();
  curv.setIsDraw(display());
  _curve.push_back(curv);

#ifdef ENABLE_DISPLAY
  viewer->curve.push_back(curv);
  viewer->camera()->setPosition(qglviewer::Vec( 0., 0., 0.5));
  viewer->updateGL();

  _plot.plotMotionLaw(curv, qwtPlot_TrajJerk, qwtPlot_TrajAcc, qwtPlot_TrajVel);
  _plot.plotIdealProfile(curv, qwtPlot_PosXideal, qwtPlot_VelXideal, qwtPlot_AccXideal,
			 qwtPlot_PosYideal, qwtPlot_VelYideal, qwtPlot_AccYideal);
#else 

  computeTraj();
  genFileTraj();

#endif
  return;  
}

/* the definition of a line y = y1 + (x-x1) * (y2-y1)/(x2-x1) */
void QSoftMotionPlanner::defineFunction_l(){
  Curve curv;

  Path lpath;
  SubPath lsubpath;
  std::string str2;
  double x1 = 0.0,y1 = 0.0;
  double x2 = 0.03,y2 = 0.02;

  lsubpath.line.start.x = x1;
  lsubpath.line.start.y = y1;
  lsubpath.line.end.x = x2;
  lsubpath.line.end.y = y2;
  lsubpath.type = LINE_TH;
  lpath.subpath.push_back(lsubpath);
  curv.path.push_back(lpath);  
  constructTrajSvg(curv.path, _sampling, _lim, curv.traj);
  
  str2.clear();
  str2 += "line_traj.dat";
  saveTraj(str2, curv.traj);
  curv.draw();
  curv.setIsDraw(display());
  _curve.push_back(curv);

#ifdef ENABLE_DISPLAY
    viewer->curve.push_back(curv);
  viewer->camera()->setPosition(qglviewer::Vec( 0., 0., 0.5));
  viewer->updateGL();

  _plot.plotMotionLaw(curv, qwtPlot_TrajJerk, qwtPlot_TrajAcc, qwtPlot_TrajVel);
  _plot.plotIdealProfile(curv, qwtPlot_PosXideal, qwtPlot_VelXideal, qwtPlot_AccXideal,
			 qwtPlot_PosYideal, qwtPlot_VelYideal, qwtPlot_AccYideal);
#else 

  computeTraj();
  genFileTraj();

#endif
  return;  
}

/* the definition of a sinusoid y = asin(2PIft + phi) */
void QSoftMotionPlanner::defineFunction_s(){
  Curve curv;
  double a1 = 0.1;
  double f1 = 10.0;
  double phi1 = 0.0;
  Path lpath;
  SubPath lsubpath;
  std::string str2;
 
  lsubpath.sinus.start.x = 0.0;
  lsubpath.sinus.start.y = 0.0;
  lsubpath.sinus.phase = phi1;
  lsubpath.sinus.amplitude = a1;
  lsubpath.sinus.frequency = f1;
  lsubpath.type = SINUS;
  lsubpath.sinus.length_x = 0.2;
  lpath.subpath.push_back(lsubpath);
  curv.path.push_back(lpath);
  constructTrajSvg(curv.path,_sampling, _lim, curv.traj);
  
  str2.clear();
  str2 += "sinus_traj.dat";
  saveTraj(str2, curv.traj);
  curv.draw();
  curv.setIsDraw(display());
  _curve.push_back(curv);

#ifdef ENABLE_DISPLAY
    viewer->curve.push_back(curv);
  viewer->camera()->setPosition(qglviewer::Vec( 0., 0., 0.5));
  viewer->updateGL();

  _plot.plotMotionLaw(curv, qwtPlot_TrajJerk, qwtPlot_TrajAcc, qwtPlot_TrajVel);
  _plot.plotIdealProfile(curv, qwtPlot_PosXideal, qwtPlot_VelXideal, qwtPlot_AccXideal,
			 qwtPlot_PosYideal, qwtPlot_VelYideal, qwtPlot_AccYideal);
#else 

  computeTraj();
  genFileTraj();

#endif
  return;
}


#ifdef ENABLE_DISPLAY
void QSoftMotionPlanner::openFile()
{
  QString fileName = QFileDialog::getOpenFileName(this);
  std::string str;
  std::string str2;

  Curve curv;

  if (!fileName.isEmpty()){
    this->_fileName = fileName.toStdString();
    cout << "Open file " << this->_fileName << endl;
    str.clear();
    str += "Soft Motion Planner : ";
    str += this->_fileName.c_str();

    setWindowTitle(QApplication::translate("QSoftMotionPlanner", str.c_str(), 0, QApplication::UnicodeUTF8));


    if (parseSvg(this->_fileName.c_str(), curv.path, &curv.width, &curv.height) == SM_ERROR) {
      return;
    }
    cout << " parse is OK" << endl;

    // Custom Params
    _lim.maxJerk = this->doubleSpinBox_Jmax->value();
    _lim.maxAcc  = this->doubleSpinBox_Amax->value();
    _lim.maxVel  = this->doubleSpinBox_Vmax->value();
    _sampling = this->doubleSpinBox_SamplingTime->value();
    _errMax =  this->doubleSpinBox_DesError->value();
    _timeStep = this->doubleSpinBoxFileSampling->value();


    QSoftMotionPlanner::initializeApproxVariables();	
    /* computation of the trajectory*/

    constructTrajSvg(curv.path,_sampling, _lim, curv.traj);
    
    str2.clear();
    str2 += "QtIdealTraj.dat";
    saveTraj(str2, curv.traj);
    
    /* Handle the path */
    curv.draw();
    curv.setIsDraw(display());

    _curve.push_back(curv);

    viewer->curve.push_back(curv);
    // cout << " size stack " << _curve.back().path.size()  << " origin " << curv.path.size() << endl;
    viewer->camera()->setPosition(qglviewer::Vec( 0., 0., 0.5));
    // cout << "nbkey " << curv.nbKeyFrames() << endl;
    viewer->updateGL();
    
    _plot.plotMotionLaw(curv, qwtPlot_TrajJerk, qwtPlot_TrajAcc, qwtPlot_TrajVel);
    _plot.plotIdealProfile(curv, qwtPlot_PosXideal, qwtPlot_VelXideal, qwtPlot_AccXideal,
			   qwtPlot_PosYideal, qwtPlot_VelYideal, qwtPlot_AccYideal);
    //     _plot.plotResults(curv, qwtPlot_JerkYapprox, qwtPlot_JerkZapprox, 
    //		       qwtPlot_AccXapprox, qwtPlot_AccYapprox, qwtPlot_AccZapprox,
    //			qwtPlot_VelXapprox, qwtPlot_VelYapprox, qwtPlot_VelZapprox,
    //		       qwtPlot_TrajPosApprox, qwtPlot_TrajVelApprox, qwtPlot_TrajAccApprox);

    
  }
}

#else

void QSoftMotionPlanner::openFile()
{
  
  std::string str;
  std::string str2;
  Curve curv;



    cout << "Open file " << this->_fileName << endl;
    str.clear();
    str += "Soft Motion Planner : ";
    str += this->_fileName.c_str();

    if (parseSvg(this->_fileName.c_str(), curv.path, &curv.width, &curv.height) == SM_ERROR) {
      return;
    }
    cout << "... file parsed " << endl;

    QSoftMotionPlanner::initializeApproxVariables();	
    /* computation of the trajectory*/
    constructTrajSvg(curv.path, _sampling, _lim, curv.traj);
    
    str2.clear();
    str2 += "QtIdealTraj.dat";
    saveTraj(str2, curv.traj);
    
    /* Handle the path */
     curv.setIsDraw(false);
    curv.draw();
    _curve.push_back(curv);
    cout << " ... Ideal Trajectory Computed " << endl;


  computeTraj();

    cout << " ... Approximated Trajectory Computed " << endl;
  genFileTraj();
    cout << " ... File exported " << endl;


}

#endif
void QSoftMotionPlanner::fullScreen()
{
#ifdef ENABLE_DISPLAY 
  if(this->_isFullScreen == false){
    this->showFullScreen();
    this->_isFullScreen = true;
  } 
  else {
    this->_isFullScreen = false;
    this->showMaximized();
  }
#endif
  return;
}

void QSoftMotionPlanner::closeFile()
{

  this->_curve.clear();
#ifdef ENABLE_DISPLAY
  this->viewer->updateGL();
  setWindowTitle(QApplication::translate("QSoftMotionPlanner","Soft Motion Planner", 0, QApplication::UnicodeUTF8));
#endif
  return;
}

void QSoftMotionPlanner::computeTraj()
{
#ifdef ENABLE_DISPLAY
    QApplication::setOverrideCursor(Qt::WaitCursor);

#endif
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
  double tu,ts;
  double tic = 0.0;
  double pas_inter = 0.0;
  double longeur_path = 0.0;
  double sum_time = 0.0;
  double val_err_max = 0.0;
  double val_err_max_traj = 0.0;
  double err_max_def = 0.0;
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
  
  ChronoOn();

  for(unsigned int i=0 ; i< _curve.size(); i++) {
  
    _curve[i].setIsDraw(display());
  
  }

  lim.maxJerk = _lim.maxJerk;
  lim.maxAcc = _lim.maxAcc;
  lim.maxVel = _lim.maxVel;
  tic = _sampling;
  err_max_def = _errMax;

  Path_Length(_curve.front().path, &longeur_path);
  pas_inter = longeur_path/10;
  nbIntervals_global = int(ceil(longeur_path/pas_inter));

  saveTraj("QtIdealTraj2.dat", _curve.begin()->traj);

  _curve.begin()->setIsDraw(display());

#ifdef ENABLE_DISPLAY
    viewer->updateGL();
#endif

  curv2.traj.clear();
  IC.resize(nbIntervals_global);
  FC.resize(nbIntervals_global);
  Timp.resize(nbIntervals_global);
  IntervIndex_global.resize(nbIntervals_global + 1);

  if(sm_ComputeCondition(_curve.front().traj, _curve.front().discPoint, IC, FC, Timp, IntervIndex_global)!=0){
    printf("QSoftMotionPlanner::computeTraj() ERROR in sm_ComputeCondition()\n");
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
      iter_temp->traj[k].t = _curve.front().traj[kk].t;
    
      iter_temp->traj[k].Pos[0] = _curve.front().traj[kk].Pos[0];
      iter_temp->traj[k].Pos[1] = _curve.front().traj[kk].Pos[1];
      iter_temp->traj[k].Pos[2] = _curve.front().traj[kk].Pos[2];
    
      iter_temp->traj[k].Vel[0] = _curve.front().traj[kk].Vel[0];
      iter_temp->traj[k].Vel[1] = _curve.front().traj[kk].Vel[1];
      iter_temp->traj[k].Vel[2] = _curve.front().traj[kk].Vel[2];
    
      iter_temp->traj[k].Acc[0] = _curve.front().traj[kk].Acc[0];
      iter_temp->traj[k].Acc[1] = _curve.front().traj[kk].Acc[1];
      iter_temp->traj[k].Acc[2] = _curve.front().traj[kk].Acc[2];
    
      iter_temp->traj[k].u = _curve.front().traj[kk].u;
      iter_temp->traj[k].du = _curve.front().traj[kk].du;
      iter_temp->traj[k].ddu = _curve.front().traj[kk].ddu;
      iter_temp->traj[k].AccNorm = _curve.front().traj[kk].AccNorm;
    
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

      if (sm_ComputeCondition(iter_temp->traj, _curve.front().discPoint, IC, FC, Timp, IntervIndex) != 0){
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
        Temp_alias.push_back((iter_temp_divis->traj.size()-1)*tic);

        sm_SolveWithoutOpt(IC_seg, FC_seg, Temp_alias, iter_divis->motion_par_seg);

        iter_divis->traj.clear();

        convertMotionToCurve(iter_divis->motion_par_seg, tic, 1, iter_divis->traj);

        if(iter_temp_divis->traj.size() <= iter_divis->traj.size()) {toto = iter_temp_divis->traj.size();}
        else {toto = iter_divis->traj.size();}
        Calcul_Error_list(iter_temp_divis->traj, iter_divis->traj, &_curve.front().errorMax, error, &val_err_max, toto);
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
      //cout << "err_max_dans_chaq_segment :  " << err_max_chaq_seg << " " << err_max_def << endl; 
    }while (err_max_chaq_seg > err_max_def);

  }
#ifdef ENABLE_DISPLAY
  _plot.plotResults(curv2, qwtPlot_JerkXapprox, qwtPlot_JerkYapprox, qwtPlot_JerkZapprox, 
		    qwtPlot_AccXapprox, qwtPlot_AccYapprox, qwtPlot_AccZapprox,
		    qwtPlot_VelXapprox, qwtPlot_VelYapprox, qwtPlot_VelZapprox,
		    qwtPlot_TrajPosApprox, qwtPlot_TrajVelApprox, qwtPlot_TrajAccApprox); 
#endif
  Calcul_Error(_curve.begin()->traj, curv2.traj, &_curve.begin()->errorMax, error_traj, &val_err_max_traj);

#ifdef ENABLE_DISPLAY
  _plot.plotErrors(curv2, error_traj, &val_err_max_traj,qwtPlot_errortraj);
#endif

  saveTraj("QtApproxTraj.dat", curv2.traj);
  curv2.setIsDraw(display());
  curv2.discPoint = _curve.front().discPoint;
  curv2.errorMax = _curve.front().errorMax;
  _curve.push_back(curv2);

#ifdef ENABLE_DISPLAY
  viewer->curve.push_back(curv2);
  viewer->updateGL();
#endif
  ChronoPrint("");
  ChronoTimes(&tu, &ts);
  ChronoOff();
#ifdef ENABLE_DISPLAY
  QApplication::setOverrideCursor(Qt::ArrowCursor);
#endif
  return;
}


void QSoftMotionPlanner::computeSoftMotion()
{
#ifdef ENABLE_DISPLAY
  SM_COND ICloc, FCloc;
  SM_LIMITS lim;
  SM_TIMES TimeSeg, TimeSegFond;
  int TrajectoryType, TrajectoryTypeFond;
  double dcOut, dcOutFond;
  int zoneOut, zoneOutFond, i;
  std::vector<double> Time;
  Time.resize(7);
  std::vector<double>  IC;
  IC.resize(3);
  std::vector<double> J;
  J.resize(7);
  double total_time=0.0;
  int nbPoints=0, nbPointsFond=0;
  double tic = 0.005;
  double tloc = 0.0;
  std::vector<double> t, u, du, ddu;
  std::vector<double> tFond, uFond, duFond, dduFond;
  //  double *t=NULL, *ddu=NULL, *du=NULL, *u=NULL;
  // double *tFond=NULL, *dduFond=NULL, *duFond=NULL, *uFond=NULL;
    



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

  ddu.resize(nbPoints);  
  du.resize(nbPoints);  
  u.resize(nbPoints); 
  t.resize(nbPoints);
  
  this->doubleSpinBox_totalTime->setValue(total_time);



  for (i = 0; i < nbPoints; i++){
    tloc = i * tic;
    if (tloc >= total_time) {
      tloc =total_time;
    }
    t.at(i) = tloc;
  }

  sm_AVX_TimeVar(IC, Time, J, t, ddu, du, u);

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


  dduFond.resize(nbPointsFond);  
  duFond.resize(nbPointsFond);  
  uFond.resize(nbPointsFond); 
  tFond.resize(nbPointsFond);


 
  for (i = 0; i < nbPointsFond; i++){
    tloc = i * tic;
    if (tloc >= total_time) {
      tloc =total_time;
    }
    tFond.at(i) = tloc;
  }

  sm_AVX_TimeVar(IC, Time, J, tFond, dduFond, duFond, uFond);

  this->qwtPlot_2->clear();
  this->qwtPlot->clear();
  this->qwtPlot->setCanvasBackground(QColor(Qt::white));
  this->qwtPlot_2->setCanvasBackground(QColor(Qt::white));


  const QVector<double> qv_t = QVector<double>::fromStdVector(t);
  const QVector<double> qv_u = QVector<double>::fromStdVector(u);
  const QVector<double> qv_du = QVector<double>::fromStdVector(du);
  const QVector<double> qv_ddu = QVector<double>::fromStdVector(ddu);  

  QwtPlotCurve *curve_pos = new QwtPlotCurve("Pos--(m)");
  QPen pen_pos = curve_pos->pen();
  pen_pos.setColor(Qt::darkYellow);
  pen_pos.setWidth(2);
  pen_pos.setStyle(Qt::DashDotLine);
  curve_pos->setPen(pen_pos);
  curve_pos->setData(qv_t ,qv_u);
  curve_pos->attach(this->qwtPlot_2);

  QwtPlotCurve *curve_vel = new QwtPlotCurve("Vel--(m/s)");
  QPen pen_vel = curve_vel->pen();
  pen_vel.setColor(Qt::darkGreen);
  pen_vel.setWidth(2);
  curve_vel->setPen(pen_vel);
  curve_vel->setData(qv_t,qv_du);
  curve_vel->attach(this->qwtPlot_2);

  QwtPlotCurve *curve_acc = new QwtPlotCurve("Acc--(m/s^2)");
  QPen pen_acc = curve_acc->pen();
  pen_acc.setColor(Qt::blue);
  pen_acc.setWidth(2);
  curve_acc->setPen(pen_acc);
  curve_acc->setData(qv_t,qv_ddu);
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

  const QVector<double> qv_duFond = QVector<double>::fromStdVector(duFond);
  const QVector<double> qv_dduFond = QVector<double>::fromStdVector(dduFond);

  QwtPlotCurve *curve_velacc = new QwtPlotCurve("Vel_Acc");
  QPen pen_velacc = curve_velacc->pen();
  pen_velacc.setColor(Qt::red);
  pen_velacc.setWidth(2);
  curve_velacc->setPen(pen_velacc);
  curve_velacc->setData(qv_du,qv_ddu);
  curve_velacc->attach(this->qwtPlot);

  QwtPlotCurve *curve_velaccFond = new QwtPlotCurve("Vel_Acc_Lim");
  QPen pen_velaccFond = curve_velaccFond->pen();
  pen_velaccFond.setColor(Qt::blue);
  pen_velaccFond.setWidth(1);
  curve_velaccFond->setPen(pen_velaccFond);
  curve_velaccFond->setData(qv_duFond,qv_dduFond);
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

 
#endif
  return;
}


