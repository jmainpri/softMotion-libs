#ifndef PLOT_H
#define PLOT_H

#include <string>
#include <iostream>
#include <fstream>
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
#include "curve.h"

class MyPlot  {

 public:
    /** @brief plot of motion law
     *  
     * plot the motion law "SoftMotion"
     *
     * @param &curv : a curve with motion law
     */   
    void plotMotionLaw(Curve &curv, QwtPlot *qwtPlot_TrajJerk, QwtPlot *qwtPlot_TrajAcc, QwtPlot *qwtPlot_TrajVel);
    
    /** @brief plot of motion profile
     *  
     * plot the ideal trajectory with "SoftMotion" in axis x and y separately
     *
     * @param &curv : a curve with motion law
     */     
    void plotIdealProfile(Curve &curv, QwtPlot * qwtPlot_PosXideal,QwtPlot *qwtPlot_VelXideal,QwtPlot *qwtPlot_AccXideal,
			  QwtPlot * qwtPlot_PosYideal,QwtPlot *qwtPlot_VelYideal,QwtPlot *qwtPlot_AccYideal);
    
    /** @brief plot of graph
     *  
     * main function for plotting the trajectory
     *
     * @param *p : pointer of an QwtPlot object 
     * @param xData[] : array for stocking the datas in axis X
     * @param yData[] : array for stocking the datas in axis Y
     * @param size : array size for xData and yData
     * @param xName : name of axis X
     * @param yName : name of axis Y
     * @param title : title of the plot
     */       
    void plotGraph(QwtPlot *p, double xData[], double yData[], int size, char* xName, char *yName, char *title);
    
    /** @brief plot of the approximated trajectory 
     *  
     * plot the approximated trajectory with "SoftMotion" in axis x and y separately 
     *
     * @param &curv2 : a curve with motion law
     */   
    void plotResults(Curve &curv2, QwtPlot *qwtPlot_JerkXapprox, QwtPlot *qwtPlot_JerkYapprox, QwtPlot *qwtPlot_JerkZapprox, 
		       QwtPlot *qwtPlot_AccXapprox, QwtPlot *qwtPlot_AccYapprox, QwtPlot *qwtPlot_AccZapprox,
			 QwtPlot *qwtPlot_VelXapprox, QwtPlot *qwtPlot_VelYapprox, QwtPlot *qwtPlot_VelZapprox,
		       QwtPlot *qwtPlot_TrajPosApprox, QwtPlot *qwtPlot_TrajVelApprox, QwtPlot *qwtPlot_TrajAccApprox);
    
    /** @brief plot of errors
     *  
     * plot the error of ideal and approximated trajectories
     *
     * @param &curv2 : a curve with motion law
     * @param &error : a vector of errors
     * @param *val_err_max : pointer of the maximum value of error
     */      
    void plotErrors(Curve &curv2, std::vector<double> &error, double *val_err_max, QwtPlot *qwtPlot_errortraj);
    
    /** @brief plot of hausdorff distance
     *  
     * plot the hausdorff distance between the ideal and approximated trajectories
     *
     * @param &dis1 : the first vector of hausdorff distance 
     * @param &dis2 : the second vector of hausdorff distance
     * @param sup1 : the maximum value of the first vector hausdorff 
     * @param sup2 : the maximum value of the second vector hausdorff
     */       
    void plotHaus(std::vector<double> &dis1, std::vector<double> &dis2, double sup1, double sup2, QwtPlot *qwtPlot_haussdorff1, QwtPlot *qwtPlot_haussdorff2);


 private:



};

#endif
