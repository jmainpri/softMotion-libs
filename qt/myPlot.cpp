


#include "softMotion.h"
#include "myPlot.h"

void MyPlot::plotMotionLaw(Curve &curv, QwtPlot *qwtPlot_TrajJerk, QwtPlot *qwtPlot_TrajAcc, QwtPlot *qwtPlot_TrajVel) {
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

void MyPlot::plotIdealProfile(Curve &curv,QwtPlot * qwtPlot_PosXideal,QwtPlot *qwtPlot_VelXideal,QwtPlot *qwtPlot_AccXideal,
			    QwtPlot * qwtPlot_PosYideal,QwtPlot *qwtPlot_VelYideal,QwtPlot *qwtPlot_AccYideal) {
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
    xData_vx_tra [i] = curv.traj[i].t;
  }
  for (int i  = 0; i < (int)curv.traj.size();i++){
    yData_ax_tra [i] = acc_discr_X[i];
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


void MyPlot::plotGraph(QwtPlot *p, double xData[], double yData[], int size, char* xName, char *yName, char *title) {
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
  p->setAutoReplot(true);
  return;

}



void MyPlot::plotErrors(Curve &curv2, std::vector<double> &error, double *val_err_max, QwtPlot *qwtPlot_errortraj) {
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


void MyPlot::plotResults(Curve &curv2, QwtPlot *qwtPlot_JerkXapprox, QwtPlot *qwtPlot_JerkYapprox, QwtPlot *qwtPlot_JerkZapprox, 
		       QwtPlot *qwtPlot_AccXapprox, QwtPlot *qwtPlot_AccYapprox, QwtPlot *qwtPlot_AccZapprox,
			 QwtPlot *qwtPlot_VelXapprox, QwtPlot *qwtPlot_VelYapprox, QwtPlot *qwtPlot_VelZapprox,
		       QwtPlot *qwtPlot_TrajPosApprox, QwtPlot *qwtPlot_TrajVelApprox, QwtPlot *qwtPlot_TrajAccApprox) {

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

  /* abscicca_approx*/
  double length_petit = 0.0;
  double length_total = 0.0;
  std::vector<double> length_vec;
  double *xData_ab_app = NULL, *yData_ab_app  = NULL;
  xData_ab_app = (double* ) malloc (sizeof(double) * curv2.traj.size());
  yData_ab_app = (double* ) malloc (sizeof(double) * curv2.traj.size());
  for (int i  = 0; i < (int)curv2.traj.size();i++){
    if (i==0){
        length_vec.push_back(0);
    }
    else{
        length_petit = sqrt(
                    pow(curv2.traj[i].Pos[0]-curv2.traj[i-1].Pos[0],2)+
                    pow(curv2.traj[i].Pos[1]-curv2.traj[i-1].Pos[1],2)+
                    pow(curv2.traj[i].Pos[2]-curv2.traj[i-1].Pos[2],2) );
        length_total += length_petit;
        length_vec.push_back(length_total);
    }
  }
  for (int i  = 0; i < (int)curv2.traj.size();i++){
    yData_ab_app [i] = length_vec[i];
    xData_ab_app [i] = curv2.traj[i].t;
  }
  /* velocity_approx*/
  double *xData_v_app = NULL, *yData_v_app  = NULL;
  xData_v_app = (double* ) malloc (sizeof(double) * curv2.traj.size());
  yData_v_app = (double* ) malloc (sizeof(double) * curv2.traj.size());
  for (unsigned int i  = 0; i < curv2.traj.size();i++){
    yData_v_app[i] = sqrt(curv2.traj[i].Vel[0] * curv2.traj[i].Vel[0] + curv2.traj[i].Vel[1] * curv2.traj[i].Vel[1]);
    xData_v_app[i] = curv2.traj[i].t;
  }
  /* accelaration_approx*/
  double *xData_a_app = NULL, *yData_a_app  = NULL;
  xData_a_app = (double* ) malloc (sizeof(double) * curv2.traj.size());
  yData_a_app = (double* ) malloc (sizeof(double) * curv2.traj.size());
  for (unsigned int i  = 0; i < curv2.traj.size();i++){
    yData_a_app[i] = sqrt(curv2.traj[i].Acc[0] * curv2.traj[i].Acc[0] + curv2.traj[i].Acc[1] * curv2.traj[i].Acc[1]);
    xData_a_app[i] = curv2.traj[i].t;
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

  plotGraph(qwtPlot_TrajPosApprox, xData_ab_app, yData_ab_app, curv2.traj.size(), (char *)"time (s)", (char *)"absicce_approx (m)", (char *)"");
  plotGraph(qwtPlot_TrajVelApprox, xData_v_app, yData_v_app, curv2.traj.size(), (char *)"time (s)", (char *)"velocity_approx (m/s)", (char *)"");
  plotGraph(qwtPlot_TrajAccApprox, xData_a_app, yData_a_app, curv2.traj.size(), (char *)"time (s)", (char *)"accelaration_approx (m/s^2)", (char *)"");

  qwtPlot_JerkXapprox->replot();
  qwtPlot_JerkYapprox->replot();
  qwtPlot_JerkZapprox->replot();
  qwtPlot_AccXapprox->replot();
  qwtPlot_AccYapprox->replot();
  qwtPlot_AccZapprox->replot();
  qwtPlot_VelXapprox->replot();
  qwtPlot_VelYapprox->replot();
  qwtPlot_VelZapprox->replot();
  qwtPlot_TrajPosApprox->replot();
  qwtPlot_TrajVelApprox->replot();
  qwtPlot_TrajAccApprox->replot();

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

  free(xData_ab_app);
  free(yData_ab_app);
  free(xData_v_app);
  free(yData_v_app);
  free(xData_a_app);
  free(yData_a_app);

  return;
}

void MyPlot::plotHaus(std::vector<double> &dis_a_tracer1, std::vector<double> &dis_a_tracer2, double sup1, double sup2,
		      QwtPlot *qwtPlot_haussdorff1, QwtPlot *qwtPlot_haussdorff2){
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

  QwtPlotCurve *haus1 = new QwtPlotCurve("haus1");
  haus1->setData(xData_haus1,yData_haus1, dis_a_tracer1.size());
  haus1->attach(qwtPlot_haussdorff1);
  QString label_haus_peak1;
  label_haus_peak1.sprintf("dis_hau_1: %.3g (m)", sup1);
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
  label_haus_peak2.sprintf("dis_hau_2: %.3g (m)", sup2);
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

