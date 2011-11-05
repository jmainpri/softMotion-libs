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
#include "Sm_Traj.h"

#include "QSoftMotionPlanner.h"
#define PI 3.14159265
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


#ifdef ENABLE_DISPLAY
    this->setupUi(this);

    setWindowTitle(QApplication::translate("QSoftMotionPlanner", "Soft Motion Planner", 0, QApplication::UnicodeUTF8));

    this->Slider_Jmax->setRange(0,200,0.001);
    this->doubleSpinBox_Jmax->setSingleStep(0.1);
    this->doubleSpinBox_Jmax->setRange(0,200);
    this->doubleSpinBox_Jmax->setDecimals(4);

    this->Slider_Amax->setRange(0,40,0.01);
    this->doubleSpinBox_Amax->setSingleStep(0.01);
    this->doubleSpinBox_Amax->setRange(0,40);
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
    this->doubleSpinBox_Vmax->setValue(0.04);
    this->doubleSpinBox_SamplingTime->setValue(0.001);

    /*desired error here*/
    this->Slider_desError->setRange(0,0.1, 0.000001);
    this->doubleSpinBox_DesError->setRange(0,0.1);
    this->doubleSpinBox_DesError->setDecimals(6);
    this->doubleSpinBox_DesError->setSingleStep(0.000001);
    this->doubleSpinBox_DesError->setValue(0.001);

    /*coordinates*/
    this->doubleSpinBox_xend->setValue(0.1);
    this->doubleSpinBox_yend->setValue(0.15);
    this->doubleSpinBox_Radius->setValue(0.1);
    this->doubleSpinBox_amplitude->setValue(0.1);
    this->doubleSpinBox_frequency->setValue(10.0);
    this->doubleSpinBox_phase->setValue(0.0);
    this->doubleSpinBox_a->setValue(5.0);

//     connect(this->doubleSpinBox_xend, SIGNAL(valueChanged(double)), this, SLOT(resetPlanner()));
//     connect(this->doubleSpinBox_yend, SIGNAL(valueChanged(double)), this, SLOT(resetPlanner()));
//     connect(this->doubleSpinBox_Radius, SIGNAL(valueChanged(double)), this, SLOT(resetPlanner()));
//     connect(this->doubleSpinBox_amplitude, SIGNAL(valueChanged(double)), this, SLOT(resetPlanner()));
//     connect(this->doubleSpinBox_frequency, SIGNAL(valueChanged(double)), this, SLOT(resetPlanner()));
//     connect(this->doubleSpinBox_phase, SIGNAL(valueChanged(double)), this, SLOT(resetPlanner()));
//     connect(this->doubleSpinBox_a, SIGNAL(valueChanged(double)), this, SLOT(resetPlanner()));
//     connect(this->doubleSpinBox_DesError, SIGNAL(valueChanged(double)), this, SLOT(resetPlanner()));
//     connect(this->doubleSpinBox_SamplingTime, SIGNAL(valueChanged(double)), this, SLOT(resetPlanner()));
//     connect(this->doubleSpinBox_Jmax, SIGNAL(valueChanged(double)), this, SLOT(resetPlanner()));
//     connect(this->doubleSpinBox_Amax, SIGNAL(valueChanged(double)), this, SLOT(resetPlanner()));
//     connect(this->doubleSpinBox_Vmax, SIGNAL(valueChanged(double)), this, SLOT(resetPlanner()));
//
//     connect(this->doubleSpinBox_xend, SIGNAL(valueChanged(double)), this, SLOT(choose_curve()));
//     connect(this->doubleSpinBox_yend, SIGNAL(valueChanged(double)), this, SLOT(choose_curve()));
//     connect(this->doubleSpinBox_Radius, SIGNAL(valueChanged(double)), this, SLOT(choose_curve()));
//     connect(this->doubleSpinBox_amplitude, SIGNAL(valueChanged(double)), this, SLOT(choose_curve()));
//     connect(this->doubleSpinBox_frequency, SIGNAL(valueChanged(double)), this, SLOT(choose_curve()));
//     connect(this->doubleSpinBox_phase, SIGNAL(valueChanged(double)), this, SLOT(choose_curve()));
//     connect(this->doubleSpinBox_a, SIGNAL(valueChanged(double)), this, SLOT(choose_curve()));
//     connect(this->doubleSpinBox_DesError, SIGNAL(valueChanged(double)), this, SLOT(choose_curve()));
//     connect(this->doubleSpinBox_SamplingTime, SIGNAL(valueChanged(double)), this, SLOT(choose_curve()));
//     connect(this->doubleSpinBox_Jmax, SIGNAL(valueChanged(double)), this, SLOT(choose_curve()));
//     connect(this->doubleSpinBox_Amax, SIGNAL(valueChanged(double)), this, SLOT(choose_curve()));
//     connect(this->doubleSpinBox_Vmax, SIGNAL(valueChanged(double)), this, SLOT(choose_curve()));

    connect(this->action_Open,SIGNAL(triggered()),this,SLOT(openFile()));
    connect(this->actionFull_screen, SIGNAL(triggered()),this,SLOT(fullScreen()));
    connect(this->action_Close_2, SIGNAL(triggered(bool)), this, SLOT(closeFile()));
    connect(this->pushButtonGenFile, SIGNAL(clicked(bool)), this, SLOT(genFileTraj()));
    connect(this->pushButtonGenPlotFile, SIGNAL(clicked(bool)), this, SLOT(genPlotFile()));
    connect(this->pushButtonComputeHauss, SIGNAL(clicked(bool)), this, SLOT(computeHausdorff()) ) ;
    connect(this->comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(resetPlanner()));
    connect(this->comboBox, SIGNAL(currentIndexChanged(int)), this, SLOT(choose_curve()));
    connect(this->pushButton_reset, SIGNAL(clicked(bool)), this, SLOT(resetPlanner()));
    connect(this->pushButtonComputeTrajApproxDivisionPriori, SIGNAL(clicked(bool)), this, SLOT(computeTrajInAdvance()));
//     connect(this->pushButtonComputeTrajApprox, SIGNAL(clicked(bool)), this, SLOT(computeTraj()));

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
      this->Slider_Vmax_3->setValue(0.15);
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

      _flag_haus_actif = 0;

  #else
//    int ExpTime = 10;
//    double jmax=0.9, amax=0.3, vmax=0.04, SampTime =0.001, ErrMax = 0.001;
//    std::string help;
//    cout << endl << "******* Please enter the inputs in order the : Jmax, Amax, Vmax, SampTime, ErrMax, ExpTime *******"
//    << endl << " Please enter 'help' for more informations " << endl;
//    cin >> jmax >> amax >> vmax >> SampTime >> ErrMax >> ExpTime;
//    cout << "Absolute file path : " << endl;
//    cin>> _fileName;
//    cout << "open the file " << _fileName << endl;
//     approximate(jmax, amax, vmax, SampTime,  ErrMax,  ExpTime, true, _fileName);
////    testCercleForFunction_Curvature_Interval(jmax, amax, vmax, SampTime,  ErrMax,  ExpTime);

  #endif
  _nbCurve = 0;
  _fileName = "";
  _isFullScreen = false;
  _curve.clear();

}

QSoftMotionPlanner::~QSoftMotionPlanner()
{
  resetPlanner();
}

#ifdef ENABLE_DISPLAY
void QSoftMotionPlanner::choose_curve(){
  int curve_num;
  _lim.maxJerk = this->doubleSpinBox_Jmax->value();
  _lim.maxAcc  = this->doubleSpinBox_Amax->value();
  _lim.maxVel  = this->doubleSpinBox_Vmax->value();
  _sampling = this->doubleSpinBox_SamplingTime->value();
  _errMax =  this->doubleSpinBox_DesError->value();
  _timeStep = this->doubleSpinBoxFileSampling->value();
  curve_num = this->comboBox->currentIndex();

  switch(curve_num){
    case 0:             break;
    case 1: defineFunction_l(); break;
    case 2: defineFunction_c(); break;
    case 3: defineFunction_s(); break;
    case 4: defineFunction_p(); break;
    case 5: defineFunction_lccl();break;
    default:            break;
  }
  viewer->camera()->setSceneCenter(qglviewer::Vec(0,0,0));
  viewer->camera()->setSceneRadius(0.2);
  viewer->camera()->showEntireScene() ;
  this->viewer->updateGL();
  return;
}
#endif


void QSoftMotionPlanner::resetPlanner(){
  _curve.clear();
  _nbCurve = 0;
  _fileName.clear();

  #ifdef ENABLE_DISPLAY
  clearPlot();
  this->viewer->curve.clear();
  this->viewer->updateGL();

  lcdNumber_Jmax->setValue(0.0);
  lcdNumber_Amax->setValue(0.0);
  lcdNumber_Vmax->setValue(0.0);
  lcdNumber_comptuationTime->setValue(0.0);
  lcdNumber_trajError->setValue(0.0);
  lcdNumber_pathError->setValue(0.0);


//   this->qwtPlot_TrajJerk->clear();
//   this->qwtPlot_TrajAcc->clear();
//   this->qwtPlot_TrajVel->clear();
//   this->qwtPlot_PosXideal->clear();
//   this->qwtPlot_VelXideal->clear();
//   this->qwtPlot_AccXideal->clear();
//   this->qwtPlot_PosYideal->clear();
//   this->qwtPlot_VelYideal->clear();
//   this->qwtPlot_AccYideal->clear();

//   this->Slider_Jmax_3->setValue(0.9);
//   this->doubleSpinBox_Jmax_3->setValue(0.9);
//   this->Slider_Amax_3->setValue(0.3);
//   this->doubleSpinBox_Amax_3->setValue(0.3);
//   this->Slider_Vmax_3->setValue(0.15);
//   this->doubleSpinBox_Vmax_3->setValue(0.15);
//   this->Slider_A0->setValue(0.0);
//   this->doubleSpinBox_A0->setValue(0.0);
//   this->Slider_V0->setValue(0.0);
//   this->doubleSpinBox_V0->setValue(0.0);
//   this->Slider_Af->setValue(0.0);
//   this->doubleSpinBox_Af->setValue(0.0);
//   this->Slider_Vf->setValue(0.0);
//   this->doubleSpinBox_Vf->setValue(0.0);
//   this->Slider_Xf->setValue(0.1);
//   this->doubleSpinBox_Xf->setValue(0.1);
//
//     this->doubleSpinBox_xend->setValue(0.1);
//     this->doubleSpinBox_yend->setValue(0.15);
//     this->doubleSpinBox_Radius->setValue(0.1);
//     this->doubleSpinBox_amplitude->setValue(0.1);
//     this->doubleSpinBox_frequency->setValue(10.0);
//     this->doubleSpinBox_phase->setValue(0.0);
//     this->doubleSpinBox_a->setValue(5.0);
//     this->doubleSpinBox_DesError->setValue(0.0001);
//     this->doubleSpinBox_SamplingTime->setValue(0.001);
//
  #endif

  return;
}


void QSoftMotionPlanner::approximate(double jmax,double amax,double vmax,double sampTime, double ErrMax, int ExpTime, bool flagExport, std::string fileName, SM_TRAJ &traj)
{
  this->approximate(jmax, amax, vmax, sampTime,  ErrMax,  ExpTime, flagExport, fileName);
  traj.clear();


  return;
}



void QSoftMotionPlanner::approximate(double jmax,double amax,double vmax,double SampTime, double ErrMax, int ExpTime, bool flagExport, std::string fileName) {

    FILE *fp_segMotion = NULL;
    double tu = 0.0,ts = 0.0;
    _lim.maxJerk = jmax;
    _lim.maxAcc  = amax;
    _lim.maxVel  = vmax;
    _sampling = SampTime;
    _errMax = ErrMax;
    _timeStep = ExpTime;
    _flag_haus_actif = 0;
    _fileName = fileName;
    SM_OUTPUT tempo_motion;

    std::string str;
    std::string str2;
    Curve curv;

    cout << "Open file " << this->_fileName << endl;

    if (parseSvg(this->_fileName.c_str(), curv.path, &curv.width, &curv.height) == SM_ERROR) {
      cout << "... file parsed " << endl;
      return;
    }

    initializeApproxVariables();
    constructTrajSvg(curv.path, _sampling, _lim, curv.traj);
    str2.clear();
    str2 += "QtIdealTraj.dat";
    saveTraj(str2, curv.traj);

    /* Handle the path */
    curv.setIsDraw(false);
    curv.draw();
    _curve.push_back(curv);
    cout << " ... Ideal Trajectory Computed " << endl;

    ChronoOn();
    computeTraj();
    cout << " ... Approximated Trajectory Computed " << endl;


    if(flagExport) {
       genFileTraj();
     cout << " ... File exported " << endl;
    }

  /* fill result */
  for (unsigned int i = 1; i < _result.size(); i++){
    for (unsigned int j = 0; j < _result.size()-i ; j++){
      if (_result[j].premier_point > _result[j+1].premier_point){
        tempo_motion = _result[j];
        _result[j] = _result[j+1];
        _result[j+1] = tempo_motion;
      }
    }
  }

  fp_segMotion = fopen("segMotion.dat", "w");
  if(fp_segMotion==NULL) {
    std::cout << " cannont open file to write the trajectory" << std::endl;
    return;
  }

  for (unsigned int i = 0; i < _result.size(); i++){
    fprintf(fp_segMotion, "%d %lf %lf %lf %lf %lf %lf %lf\n", _result[i].premier_point, 
	    (_result[i].premier_point + 1) * _sampling, 
	    _result[i].Jerk[0], _result[i].Jerk[1], _result[i].Jerk[2], 
	    _result[i].Time[0], _result[i].Time[1], _result[i].Time[2]);
  }

  fclose(fp_segMotion);
  cout << "motion file exported" << endl;

  ChronoTimes(&tu, &ts);
  ChronoPrint("");
  ChronoOff();
  return ;
}


void QSoftMotionPlanner::genPlotFile(){
#ifdef ENABLE_DISPLAY
  QApplication::setOverrideCursor(Qt::WaitCursor);
#endif
  int incr = _timeStep;
  FILE *fp_SmCurves = NULL;
  FILE *fp_SmDiscr = NULL;
  FILE *fp_SmMaxError = NULL;
  fp_SmCurves = fopen("SmCurves.dat", "w");
  fp_SmDiscr = fopen("SmDiscr.dat", "w");
  fp_SmMaxError = fopen("SmMaxError.dat", "w");
  if((fp_SmCurves==NULL)||(fp_SmDiscr==NULL)||(fp_SmMaxError==NULL)) {
    std::cerr << " cannont open file to write the trajectory" << std::endl;
    return;
  }
  if (_flag_haus_actif == 0){
    _err_haus1.resize(_curve.back().traj.size());
    _err_haus2.resize(_curve.back().traj.size());
  }

//   Calcul_Error_Vilocity(_curve.front().traj, _curve.back().traj, _err_vit, &errMax_pos_subTraj_vit);

  for (unsigned int i = 0; i < _curve.back().traj.size(); i += incr){
    fprintf(fp_SmCurves, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", i, i*_sampling, _curve.front().traj[i].Pos[0], _curve.front().traj[i].Pos[1], _curve.front().traj[i].Pos[2], _curve.front().traj[i].Vel[0], _curve.front().traj[i].Vel[1], _curve.front().traj[i].Vel[2], _curve.front().traj[i].Acc[0], _curve.front().traj[i].Acc[1], _curve.front().traj[i].Acc[2], _curve.back().traj[i].Pos[0], _curve.back().traj[i].Pos[1], _curve.back().traj[i].Pos[2], _curve.back().traj[i].Vel[0], _curve.back().traj[i].Vel[1], _curve.back().traj[i].Vel[2], _curve.back().traj[i].Acc[0], _curve.back().traj[i].Acc[1], _curve.back().traj[i].Acc[2], _err_traj[i],  (_curve.front().traj[i].du-_curve.back().traj[i].du), _err_haus1[i], _err_haus2[i],  _curve.front().traj[i].ddu_Mlaw,  _curve.front().traj[i].du_Mlaw,  _curve.front().traj[i].u_Mlaw, _curve.front().traj[i].ddu, _curve.front().traj[i].du, _curve.front().traj[i].u, _curve.back().traj[i].ddu, _curve.back().traj[i].du, _curve.back().traj[i].u);
  }
  fclose(fp_SmCurves);

  for (unsigned int i = 0; i < _vec_kinpoint.size(); i ++){
    fprintf(fp_SmDiscr, "%d %lf %lf %lf %lf\n", 1+int(_vec_kinpoint[i].t/_sampling), _vec_kinpoint[i].t, _vec_kinpoint[i].kc[0].x, _vec_kinpoint[i].kc[1].x, _vec_kinpoint[i].kc[2].x);
  }
  fclose(fp_SmDiscr);

  fprintf(fp_SmMaxError, "%d %lf %lf %lf %lf\n", 1 + int(_curve.front().errorMax.t/_sampling), _curve.front().errorMax.t,  _curve.front().errorMax.kc[0].x, _curve.front().errorMax.kc[1].x, _curve.front().errorMax.kc[2].x);
  fclose(fp_SmMaxError);

#ifdef ENABLE_DISPLAY
  QApplication::setOverrideCursor(Qt::ArrowCursor);
#endif



// SmDiscr index t x y z
// SmMaxError index t x y z
  return;
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
  _flag_haus_actif = 1;
#ifdef ENABLE_DISPLAY
  QApplication::setOverrideCursor(Qt::WaitCursor);
#endif
//   std::vector<double> dis_a_tracer1;
//   std::vector<double> dis_a_tracer2;

  double sup1 = 0.0;
  double sup2 = 0.0;
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
    _err_haus1.push_back(inf1);
  }
  sup1 = _err_haus1[0];
  for (int m=0; m<(int)_curve.front().traj.size(); m++){
    if (_err_haus1[m]>(sup1)) {sup1 = _err_haus1[m];}
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
    _err_haus2.push_back(inf2);
  }
  sup2 = _err_haus2[0];
  for (int m=0; m<(int)_curve.back().traj.size() ; m++){
    if (_err_haus2[m]>(sup2)) {sup2 = _err_haus2[m];}
  }

  // calcul de la distance hausdorff
  dis_hausdorff = (sup1 > sup2 ? sup1 : sup2);
  cout << "Hausdorff distance = " << dis_hausdorff << endl;
#ifdef ENABLE_DISPLAY
  _plot.plotHaus(_err_haus1, _err_haus2, sup1, sup2, qwtPlot_haussdorff1, qwtPlot_haussdorff2);
  lcdNumber_pathError->setValue(dis_hausdorff);
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

//void QSoftMotionPlanner::loadSvgFile(std::string str)
//{
//  Curve curv;
//  this->_curve.push_back(curv);
//}

void QSoftMotionPlanner::initializeApproxVariables()
{
  this->_curve.clear();
#ifdef ENABLE_DISPLAY
  this->viewer->updateGL();
#endif

}

void QSoftMotionPlanner::defineFunction_lccl(){
  Curve curv;
  Path lpath;
  SubPath lsubpath1,lsubpath2,lsubpath3,lsubpath4;
  std::string str2;

  lsubpath1.lccl.line1_end.x = 1.0;
  lsubpath1.lccl.line1_end.y = 0.0;
  lsubpath1.lccl.line1_end.z = 0.0;
  lsubpath1.type = LCCL_L1;
  lpath.subpath.push_back(lsubpath1);
  lsubpath2.lccl.cercle1_rayon = 1.0;
  lsubpath2.type = LCCL_C1;
  lpath.subpath.push_back(lsubpath2);
  lsubpath3.lccl.cercle2_rayon = 1.0;
  lsubpath3.type = LCCL_C2;
  lpath.subpath.push_back(lsubpath3);
  lsubpath4.lccl.line2_end.x = 2.0;
  lsubpath4.lccl.line2_end.y = 0.0;
  lsubpath4.lccl.line1_end.z = 2.0;
  lsubpath4.type = LCCL_L2;
  lpath.subpath.push_back(lsubpath4);
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

void QSoftMotionPlanner::defineFunction_p(){
  Curve curv;
  Path lpath;
  SubPath lsubpath;
  std::string str2;
  #ifdef ENABLE_DISPLAY
  double a = this->doubleSpinBox_a->value();
  #else
  double a = 5.0;
  #endif
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


void QSoftMotionPlanner::testCercleForFunction_Curvature_Interval(double jmax,double amax,double vmax,double SampTime, double ErrMax, int ExpTime){
  _lim.maxJerk = jmax;
  _lim.maxAcc  = amax;
  _lim.maxVel  = vmax;
  _sampling = SampTime;
  _errMax = ErrMax;
  _timeStep = ExpTime;
  double tu = 0.0,ts = 0.0;
  FILE *fp_inter_curva_circle = NULL;
  std::vector<double> courbure_cercle; // stock the curvatures of different circles
  int nb_test = 100;
  ChronoOn();
  for (int i = 0; i < nb_test; i++){
    double r = 0.01;
    double f = 20.0;
    Curve curv;
    Path lpath;
    SubPath lsubpath;

    _rayon_circle_for_courbure = r * (1+i); // raise the radius of circle by step of 0.001m
    lsubpath.cercle.center.x = 0.0;
    lsubpath.cercle.center.y = 0.0;
    lsubpath.cercle.radius = _rayon_circle_for_courbure;
    lsubpath.cercle.sinus_para.frequency = f;
    lsubpath.type = CERCLE;
    lpath.subpath.push_back(lsubpath);
    curv.path.push_back(lpath);
    constructTrajSvg(curv.path, _sampling, _lim, curv.traj);

    curv.draw();
    curv.setIsDraw(display());
    _curve.push_back(curv);

    computeTraj();
    courbure_cercle.push_back(1.0/_rayon_circle_for_courbure);
    _vec_interval_courbure.push_back(_interval_courbure);
  }

  fp_inter_curva_circle = fopen("intervale_courbure.dat", "w");
  if(fp_inter_curva_circle==NULL) {
    std::cout << " cannont open file to write the interval_curvature" << std::endl;
    return;
  }

  for (int i = 0; i < nb_test; i++){
    fprintf(fp_inter_curva_circle, "%lf %lf\n", courbure_cercle[i], _vec_interval_courbure[i]);
  }

  fclose(fp_inter_curva_circle);
  cout << "interval_curvature file exported" << endl;

  ChronoTimes(&tu, &ts);
  ChronoPrint("");
  ChronoOff();
  return;
}


/* the definition of a cercle x = acos(2PIft); y = asin(2PIft) */
void QSoftMotionPlanner::defineFunction_c(){
  Curve curv;
  Path lpath;
  SubPath lsubpath;
  std::string str2;
  #ifdef ENABLE_DISPLAY
  double a1 = this->doubleSpinBox_Radius->value();
  #else
  double a1 = 0.1;
  #endif
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
  #ifdef ENABLE_DISPLAY
  double x2 = this->doubleSpinBox_xend->value();
  double y2 = this->doubleSpinBox_yend->value();
  #else
  double x2 = 0.03,y2 = 0.02;
  #endif
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
  Path lpath;
  SubPath lsubpath;
  std::string str2;
  #ifdef ENABLE_DISPLAY
  double a1 = this->doubleSpinBox_amplitude->value();
  double f1 = this->doubleSpinBox_frequency->value();
  double phi1 = this->doubleSpinBox_phase->value();
  #else
  double a1 = 0.1;
  double f1 = 10.0;
  double phi1 = 0.0;
  #endif
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
//     changeProfile(curv.traj);

    str2.clear();
    str2 += "QtIdealTraj.dat";
    saveTraj(str2, curv.traj);

    /* Handle the path */
    curv.draw();
    curv.setIsDraw(display());
    _curve.push_back(curv);
    viewer->curve.push_back(curv);
    viewer->camera()->setPosition(qglviewer::Vec( 0., 0., 0.5));
    viewer->updateGL();

    _plot.plotMotionLaw(curv, qwtPlot_TrajJerk, qwtPlot_TrajAcc, qwtPlot_TrajVel);
    _plot.plotIdealProfile(curv, qwtPlot_PosXideal, qwtPlot_VelXideal, qwtPlot_AccXideal,
               qwtPlot_PosYideal, qwtPlot_VelYideal, qwtPlot_AccYideal);
    //     _plot.plotResults(curv, qwtPlot_JerkYapprox, qwtPlot_JerkZapprox,
    //             qwtPlot_AccXapprox, qwtPlot_AccYapprox, qwtPlot_AccZapprox,
    //          qwtPlot_VelXapprox, qwtPlot_VelYapprox, qwtPlot_VelZapprox,
    //             qwtPlot_TrajPosApprox, qwtPlot_TrajVelApprox, qwtPlot_TrajAccApprox);


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

//    initializeApproxVariables();
    /* computation of the trajectory*/
    constructTrajSvg(curv.path, _sampling, _lim, curv.traj);
//     changeProfile(curv.traj);

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

// void QSoftMotionPlanner::changeProfile(std::vector<SM_CURVE_DATA>  &traj){
//   calcul_courbure(traj, _courbure);
//   for (unsigned int i = 0; i < traj.size(); i++){
//     if (traj[i].du * traj[i].du * _courbure[i] > traj[i].ddu){
//       traj[i].du =
//     }
//   }
/*

  return;
}*/

void QSoftMotionPlanner::closeFile()
{
  this->_curve.clear();
#ifdef ENABLE_DISPLAY
  this->viewer->updateGL();
  setWindowTitle(QApplication::translate("QSoftMotionPlanner","Soft Motion Planner", 0, QApplication::UnicodeUTF8));
#endif
  return;
}

void QSoftMotionPlanner::computeTrajInAdvance(){

  int nbSegment = 0;
  double tu = 0.0,ts = 0.0; // chrono on et off
  double tic = 0.0;
  double time_total = 0.0;
  double max_acc = 0.0;
  double max_vel = 0.0;
  double max_jerk = 0.0;
  double errMax_pos_traj = 0.0;
  SM_LIMITS lim;
//   kinPoint kc_subTraj;

  Curve curv2; //approximated curve

  std::vector<int> IntervIndex;
  std::vector<double> Timp;
  std::vector<double> error;
  std::vector<double> dis_a_tracer1;
  std::vector<double> dis_a_tracer2;
  std::vector<SM_COND_DIM3> IC;
  std::vector<SM_COND_DIM3> FC;
  std::vector<SM_OUTPUT> motion;

  ChronoOn();
  for(unsigned int i=0 ; i< _curve.size(); i++) {
    _curve[i].setIsDraw(display());
  }

  lim.maxVel    = _lim.maxVel;
  lim.maxAcc    = _lim.maxAcc;
  lim.maxJerk   = _lim.maxJerk;
  tic = _sampling;
  _curve.begin()->setIsDraw(display());
  time_total = (_curve.front().traj.size()-1) * tic;

  #ifdef ENABLE_DISPLAY
    viewer->updateGL();
  #endif

  curv2.traj.clear();
  curv2.traj.resize(_curve.front().traj.size());
  nbSegment = 10; // par default
  IC.resize(nbSegment);
  FC.resize(nbSegment);
  Timp.resize(nbSegment);
  IntervIndex.resize(nbSegment + 1);
  motion.resize(nbSegment*3);

  sm_ComputeCondition(_curve.begin()->traj, _curve.begin()->discPoint, IC, FC, Timp, IntervIndex);
  sm_SolveWithoutOpt(IC, FC, Timp, motion);
  convertMotionToCurve_InAdvance(motion, tic, nbSegment, curv2.traj);
  maxProfile(curv2.traj, &max_jerk, &max_acc, &max_vel);
  for (unsigned int i = 0; i < _curve.begin()->discPoint.size(); i++){
    _vec_kinpoint.push_back(_curve.begin()->discPoint[i]);
  }

  #ifdef ENABLE_DISPLAY
  _plot.plotResults(curv2, qwtPlot_JerkXapprox, qwtPlot_JerkYapprox, qwtPlot_JerkZapprox,
            qwtPlot_AccXapprox, qwtPlot_AccYapprox, qwtPlot_AccZapprox,
            qwtPlot_VelXapprox, qwtPlot_VelYapprox, qwtPlot_VelZapprox,
            qwtPlot_TrajPosApprox, qwtPlot_TrajVelApprox, qwtPlot_TrajAccApprox);
  #endif
  Calcul_Error(_curve.begin()->traj, curv2.traj, &_curve.begin()->errorMax, _err_traj, &errMax_pos_traj);

  #ifdef ENABLE_DISPLAY
    lcdNumber_trajError->setValue(errMax_pos_traj);
    lcdNumber_Jmax->setValue(max_jerk);
    lcdNumber_Amax->setValue(max_acc);
    lcdNumber_Vmax->setValue(max_vel);
    lcdNumber_nb3segInterval->setValue(nbSegment);
  #endif

  #ifdef ENABLE_DISPLAY
    _plot.plotErrors(curv2, _err_traj, &errMax_pos_traj,qwtPlot_errortraj);
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
  ChronoTimes(&tu, &ts);
  #ifdef ENABLE_DISPLAY
    lcdNumber_comptuationTime->setValue(tu);
    ChronoPrint("");
  #endif
  ChronoOff();
  #ifdef ENABLE_DISPLAY
    QApplication::setOverrideCursor(Qt::ArrowCursor);
  #endif

  return;
}

void QSoftMotionPlanner::computeTraj()
{
#ifdef ENABLE_DISPLAY
    QApplication::setOverrideCursor(Qt::WaitCursor);
#endif

  SM_LIMITS lim;
  SM_OUTPUT outMotion;
  int premier_point_motionSeg;
  int nbIntervals_local = 0;
  int kkk = 0;
  int hh = 0;
  int nouveau_temp_divis = 0; //counter for the sub-trajectory
  int size_segment = 0; //size of each sub-trajectory
  int flag_sum = 0; //it shows whether we need to divise a sub-trajectory
  int starting_point_each_seg = 0; // the starting points of each sub-trajectory
  int nb_seg_total = 0;
  int test_for_circle_only = 0;
  double tu = 0.0,ts = 0.0; // chrono on et off
  double tic = 0.0;
  double time_total = 0.0;
  double errMax_pos_subTraj = 0.0; //maxi error of each sub-trajectory
  double errMax_vel_subTraj = 0.0;
  double errMax_pos_traj = 0.0;//maxi error of the whole trajectory

  double err_max_def_pos = 0.0;
  double err_max_def_vel = 0.0;
  double err_max_pos_chaq_seg = 0.0;
  double err_max_vel_chaq_seg = 0.0;
  double max_acc = 0.0;
  double max_vel = 0.0;
  double max_jerk = 0.0;
  kinPoint3 kc_subTraj;

  Curve curv2; //approximated curve
  Curve curv_temp; //ideal curve for the iteration
  Curve curv_divis; //approximated and divised curve
  Curve curv_temp_divis; //ideal and divised curve
  Curve curv_stock; // curve for the divised trajectory
  Curve curv_temp_cond; //curve just for calculate the IC and FC

  std::vector<SM_COND_DIM3> IC;
  std::vector<SM_COND_DIM3> FC;
  std::vector<double> Timp;
  std::vector<int> IntervIndex;
  std::vector<SM_OUTPUT> motion;
  std::vector<double> error;
  std::vector<double> error_vel;
  std::list<SubTraj3>::iterator iter_temp;
  std::list<SubTraj3>::iterator iter_stock;
  ChronoOn();

  for(unsigned int i=0 ; i< _curve.size(); i++) {
    _curve[i].setIsDraw(display());
  }

  lim.maxJerk = _lim.maxJerk;
  lim.maxAcc = _lim.maxAcc;
  lim.maxVel = _lim.maxVel;
  tic = _sampling;
  err_max_def_pos = _errMax;
  err_max_def_vel = lim.maxVel;

//   Path_Length(_curve.front().path, &longeur_path);

//  saveTraj("QtIdealTraj2.dat", _curve.begin()->traj);
  _curve.begin()->setIsDraw(display());
  time_total = (_curve.front().traj.size()-1) * tic;

#ifdef ENABLE_DISPLAY
    viewer->updateGL();
#endif

  curv2.traj.clear();
  curv2.traj.resize(_curve.front().traj.size());
  nbIntervals_local = 1; /* at the beginning there is only 1 subTraj */
  error.clear();
  error_vel.clear();
  errMax_pos_subTraj = 0.0;
  SM_CURVE_DATA3 curv_donne;

  std::vector<double> Temp_alias(1);
  curv_temp_divis.trajList.resize(nbIntervals_local);
  curv_temp_divis.trajList.front().traj.resize(_curve.front().traj.size());
  curv_temp_cond.trajList.resize(1);
  std::list<SubTraj3>::iterator iter_temp_cond;
  std::list<SubTraj3>::iterator iter_temp_divis;


  /* copy the entire ideal curve into the subtraj list:
     at the beginning there is only one subTraj */
  for (iter_temp_divis = curv_temp_divis.trajList.begin(); iter_temp_divis != curv_temp_divis.trajList.end(); iter_temp_divis ++){
    iter_temp_divis->point_depart = 0;
    iter_temp_divis->traj = _curve.front().traj;
  }

  do{
    /****************************/
    /* initialize the variables */
    /****************************/
    flag_sum = 0;
    nouveau_temp_divis = 0;
    std::list<SubTraj3>::iterator iter_divis;
    curv_stock.trajList.resize(nbIntervals_local*2);
    curv_temp_cond.trajList.resize(curv_temp_divis.trajList.size());
    for(iter_stock = curv_stock.trajList.begin(); iter_stock != curv_stock.trajList.end(); iter_stock ++){
      iter_stock->traj.clear();
      iter_stock->flag_traj = -1;
    }
    for(iter_temp_cond = curv_temp_cond.trajList.begin(); iter_temp_cond != curv_temp_cond.trajList.end(); iter_temp_cond ++){
      iter_temp_cond->traj.clear();
    }
    iter_stock = curv_stock.trajList.begin();
    iter_temp_cond=curv_temp_cond.trajList.begin();

    /************************************************/
    /* copy the curv_temp_divis into curv_temp_cond */
    /************************************************/
    for (iter_temp_divis = curv_temp_divis.trajList.begin(); iter_temp_divis != curv_temp_divis.trajList.end(); iter_temp_divis ++){
      for(unsigned int kk = 0; kk < iter_temp_divis->traj.size(); kk ++){
        curv_donne = iter_temp_divis->traj[kk];
        iter_temp_cond->traj.push_back(curv_donne);
      }
      iter_temp_cond ++;
    }

    iter_temp_divis = curv_temp_divis.trajList.begin();
    curv_divis.trajList.resize(nbIntervals_local);
    iter_divis = curv_divis.trajList.begin();
    for(iter_temp_cond=curv_temp_cond.trajList.begin(); iter_temp_cond != curv_temp_cond.trajList.end(); iter_temp_cond++){
      kkk = 0;
      err_max_pos_chaq_seg = 0.0;
      err_max_vel_chaq_seg = 0.0;
      IC.resize(1);
      FC.resize(1);
      Timp.resize(1);
      IntervIndex.resize(2);

      if (sm_ComputeCondition(iter_temp_cond->traj, _curve.front().discPoint, IC, FC, Timp, IntervIndex) != 0){
    printf("Compute Problem \n");   // ici discpoint marche pas !
    return;
      }


      std::vector<SM_COND_DIM3> IC_seg(1);
      std::vector<SM_COND_DIM3> FC_seg(1);

    error.clear();
    error_vel.clear();
    memcpy(IC_seg[0].Axis, IC[hh].Axis, sizeof(SM_COND_DIM3));
    memcpy(FC_seg[0].Axis, FC[hh].Axis, sizeof(SM_COND_DIM3));
    iter_divis->motion_par_seg.resize(3);
    Temp_alias.clear();
    Temp_alias.push_back((iter_temp_divis->traj.size()-1)*tic);

    /***********************************/
    /* approximate the current subTraj */
    /***********************************/
    sm_SolveWithoutOpt(IC_seg, FC_seg, Temp_alias, iter_divis->motion_par_seg);
    iter_divis->traj.clear();
    convertMotionToCurve3(iter_divis->motion_par_seg, tic, 1, iter_divis->traj);

    /****************************************/
    /* compute error of the current subTraj */
    /****************************************/
   // iter_temp_divis->traj --> the ideal traj
   // iter_divis->traj --> the approximated traj
    Calcul_Error_list(iter_temp_divis->traj, iter_divis->traj, &_curve.front().errorMax, error, error_vel, &errMax_pos_subTraj, &errMax_vel_subTraj);


    if ( (errMax_pos_subTraj > err_max_def_pos) || (errMax_vel_subTraj > err_max_def_vel) ){
      /* the trajectory error is too high */
      for (int i = 0; i < 2; i++){
        if(i == 0){
          size_segment = int(iter_temp_divis->traj.size()/2);
          iter_stock->point_depart = iter_temp_divis->point_depart;
        }
        else{
          iter_stock->point_depart = iter_temp_divis->point_depart + size_segment;
          size_segment = iter_temp_divis->traj.size() - size_segment;
        }

        /* recopy the ideal subTraj */
        for (int k=0; k< size_segment; k++) {
          curv_donne = iter_temp_divis->traj[kkk];
          iter_stock->traj.push_back(curv_donne);
          kkk ++;
        }

        /* iter_stock->flag_traj = 1 --> the error is BAD */
        iter_stock->flag_traj = 1;
        iter_stock++;
        nouveau_temp_divis ++;
      }
    }
    else{
      /* save the approximated subTraj into curv_stock */
      if(test_for_circle_only == 0){
        _interval_courbure = 2 * PI * _rayon_circle_for_courbure / nbIntervals_local ;
        test_for_circle_only = 1;
      }
      iter_stock->point_depart = iter_temp_divis->point_depart;
      premier_point_motionSeg = iter_temp_divis->point_depart;
      size_segment = iter_divis->traj.size();
      for(int nb_motionSeg = 0; nb_motionSeg < 3; nb_motionSeg ++){
        outMotion.premier_point = premier_point_motionSeg + nb_motionSeg * int(iter_divis->motion_par_seg[nb_motionSeg].Time[0]/_sampling);
        outMotion.Jerk = iter_divis->motion_par_seg[nb_motionSeg].Jerk;
        outMotion.Time = iter_divis->motion_par_seg[nb_motionSeg].Time;
        _result.push_back(outMotion);
      }
      for (int k = 0; k < size_segment; k++){
        curv_donne = iter_divis->traj[k];
        iter_stock->traj.push_back(curv_donne);
      }

       /* iter_stock->flag_traj = 0 the error is OK */
      iter_stock->flag_traj = 0;
      iter_stock++;
    }
    iter_divis++;
    iter_temp_divis ++;
  } /* for all subTraj in curv_temp_cond */

    /**********************************************************************************/
    /* here one loop of the trajectory approximation is done --> saved in curv_stock  */
    /*in curv_stock there are approximated and ideal (where error is too high) subTraj*/
    /**********************************************************************************/

    /* re-initialize the variables */
    flag_sum = 0;
    nbIntervals_local = nouveau_temp_divis;
    curv_temp_divis.trajList.resize(nbIntervals_local);
    for(iter_temp_divis = curv_temp_divis.trajList.begin(); iter_temp_divis != curv_temp_divis.trajList.end(); iter_temp_divis ++){
      iter_temp_divis->traj.clear();
    }
    iter_temp_divis = curv_temp_divis.trajList.begin();


    /*****************************************************************/
    /* recopy in curv_temp_divis the subTraj that need to be divided */
    /* recopy in curv2 the subTraj that are OK                       */
    /*****************************************************************/
    for (iter_stock = curv_stock.trajList.begin(); iter_stock != curv_stock.trajList.end(); iter_stock++){
      if (iter_stock->flag_traj == 1){
        /* the error of this subTraj is too high --> need to be divided */
        iter_temp_divis->point_depart = iter_stock->point_depart;
        for (unsigned int k = 0; k < iter_stock->traj.size(); k++) {
          curv_donne = iter_stock->traj[k];
          iter_temp_divis->traj.push_back(curv_donne);
        }

        iter_temp_divis ++;
        flag_sum ++;
      }
      else if (iter_stock->flag_traj == 0){
        nb_seg_total ++;
        starting_point_each_seg = iter_stock->point_depart;
        for (unsigned int k = starting_point_each_seg; k < (iter_stock->traj.size() + starting_point_each_seg); k++) {
          if((int)k == starting_point_each_seg){
            kc_subTraj.t = k * tic;
            kc_subTraj.kc[0].x = iter_stock->traj[k-starting_point_each_seg].Pos[0];
            kc_subTraj.kc[1].x = iter_stock->traj[k-starting_point_each_seg].Pos[1];
            kc_subTraj.kc[2].x = iter_stock->traj[k-starting_point_each_seg].Pos[2];
            _vec_kinpoint.push_back(kc_subTraj);
          }

           curv2.traj[k] = iter_stock->traj[k-starting_point_each_seg];
           curv2.traj[k].t = k * tic;
        } /* end for copy value in curv2 */
      } /* end else if (iter_stock->flag_traj == 0) -->  err < errMax*/

    }



  }while(flag_sum != 0);

  maxProfile(curv2.traj, &max_jerk, &max_acc, &max_vel);

#ifdef ENABLE_DISPLAY
  _plot.plotResults(curv2, qwtPlot_JerkXapprox, qwtPlot_JerkYapprox, qwtPlot_JerkZapprox,
            qwtPlot_AccXapprox, qwtPlot_AccYapprox, qwtPlot_AccZapprox,
            qwtPlot_VelXapprox, qwtPlot_VelYapprox, qwtPlot_VelZapprox,
            qwtPlot_TrajPosApprox, qwtPlot_TrajVelApprox, qwtPlot_TrajAccApprox);
#endif
  Calcul_Error(_curve.begin()->traj, curv2.traj, &_curve.begin()->errorMax, _err_traj, &errMax_pos_traj);
#ifdef ENABLE_DISPLAY
  lcdNumber_trajError->setValue(errMax_pos_traj);
  lcdNumber_Jmax->setValue(max_jerk);
  lcdNumber_Amax->setValue(max_acc);
  lcdNumber_Vmax->setValue(max_vel);
  lcdNumber_nb3segInterval->setValue(nb_seg_total);
#endif

#ifdef ENABLE_DISPLAY
  _plot.plotErrors(curv2, _err_traj, &errMax_pos_traj,qwtPlot_errortraj);
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
  ChronoTimes(&tu, &ts);
#ifdef ENABLE_DISPLAY
  lcdNumber_comptuationTime->setValue(tu);
 ChronoPrint("");
  #endif
  ChronoOff();
#ifdef ENABLE_DISPLAY
  QApplication::setOverrideCursor(Qt::ArrowCursor);
#endif
  return;
}





void QSoftMotionPlanner::maxProfile(std::vector<SM_CURVE_DATA3>  &ApproxTraj, double *max_jerk, double *max_acc, double *max_vel){
  double v = 0.0;
  double a = 0.0;
  double j = 0.0;
  std::vector<double> vel_norm_vec;
  std::vector<double> acc_norm_vec;
  std::vector<double> jerk_norm_vec;
  for(unsigned int i = 0; i < ApproxTraj.size(); i++){
    v = sqrt(
        ApproxTraj[i].Vel[0] * ApproxTraj[i].Vel[0] +
        ApproxTraj[i].Vel[1] * ApproxTraj[i].Vel[1] +
        ApproxTraj[i].Vel[2] * ApproxTraj[i].Vel[2]
        );
    a = sqrt(
        ApproxTraj[i].Acc[0] * ApproxTraj[i].Acc[0] +
        ApproxTraj[i].Acc[1] * ApproxTraj[i].Acc[1] +
        ApproxTraj[i].Acc[2] * ApproxTraj[i].Acc[2]
        );
    j = sqrt(
        ApproxTraj[i].Jerk[0] * ApproxTraj[i].Jerk[0] +
        ApproxTraj[i].Jerk[1] * ApproxTraj[i].Jerk[1] +
        ApproxTraj[i].Jerk[2] * ApproxTraj[i].Jerk[2]
        );
    jerk_norm_vec.push_back(j);
    acc_norm_vec.push_back(a);
    vel_norm_vec.push_back(v);
  }

  *max_vel = vel_norm_vec[0];
  *max_acc = acc_norm_vec[0];
  *max_jerk = jerk_norm_vec[0];
  for(unsigned int i = 0; i < ApproxTraj.size(); i++){
    if(*max_vel < vel_norm_vec[i])     { *max_vel = vel_norm_vec[i]; }
    if(*max_acc < acc_norm_vec[i])     { *max_acc = acc_norm_vec[i]; }
    if(*max_jerk < jerk_norm_vec[i])   { *max_jerk = jerk_norm_vec[i]; }
  }

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

#ifdef ENABLE_DISPLAY
void QSoftMotionPlanner::clearPlot(){
    qwtPlot_TrajJerk->clear();
    qwtPlot_TrajJerk->replot();
    qwtPlot_TrajAcc->clear();
    qwtPlot_TrajAcc->replot();
    qwtPlot_TrajVel->clear();
    qwtPlot_TrajVel->replot();

    qwtPlot_TrajAccApprox->clear();
    qwtPlot_TrajAccApprox->replot();
    qwtPlot_TrajVelApprox->clear();
    qwtPlot_TrajVelApprox->replot();
    qwtPlot_TrajPosApprox->clear();
    qwtPlot_TrajPosApprox->replot();

    qwtPlot_JerkXapprox->clear();
    qwtPlot_JerkXapprox->replot();
    qwtPlot_AccXapprox->clear();
    qwtPlot_AccXapprox->replot();
    qwtPlot_VelXapprox->clear();
    qwtPlot_VelXapprox->replot();

    qwtPlot_JerkYapprox->clear();
    qwtPlot_JerkYapprox->replot();
    qwtPlot_AccYapprox->clear();
    qwtPlot_AccYapprox->replot();
    qwtPlot_VelYapprox->clear();
    qwtPlot_VelYapprox->replot();

    qwtPlot_JerkZapprox->clear();
    qwtPlot_JerkZapprox->replot();
    qwtPlot_AccZapprox->clear();
    qwtPlot_AccZapprox->replot();
    qwtPlot_VelZapprox->clear();
    qwtPlot_VelZapprox->replot();

    qwtPlot_AccXideal->clear();
    qwtPlot_AccXideal->replot();
    qwtPlot_VelXideal->clear();
    qwtPlot_VelXideal->replot();
    qwtPlot_PosXideal->clear();
    qwtPlot_PosXideal->replot();

    qwtPlot_AccYideal->clear();
    qwtPlot_AccYideal->replot();
    qwtPlot_VelYideal->clear();
    qwtPlot_VelYideal->replot();
    qwtPlot_PosYideal->clear();
    qwtPlot_PosYideal->replot();

    qwtPlot_haussdorff1->clear();
    qwtPlot_haussdorff1->replot();
    qwtPlot_haussdorff2->clear();
    qwtPlot_haussdorff2->replot();
    qwtPlot_errortraj->clear();
    qwtPlot_errortraj->replot();


   return;
}
#endif

