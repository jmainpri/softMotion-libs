#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <string>
#include "curve.h"
#include "ui_mainwindow.h"
#include "gbM/gbGENOM.h"

/** @file mainwindow.h
 * @brief Declaration of the MainWindow class
 */

namespace Ui {
    class MainWindow;
}

/** @brief API of SoftMotion
 *
 */
class MainWindow :  public QMainWindow, public Ui_MainWindow  {
    Q_OBJECT
    
public:
    /** @brief Constructor
     *  
     * The constructor of class MainWindow
     *
     * @param *parent : Null
     */
    MainWindow(QWidget *parent = 0);
    
    /** @brief Destructor
     *  
     * The destructor of class MainWindow
     */
    ~MainWindow();
    
    /** @brief load the file
     *  
     * load a file .svg
     *
     * @param str : file name
     */    
    void loadSvgFile(std::string str);
    
    /** @brief definition of lines
     *  
     * represent a line defined by y = y0 + (x-x0) * (y1-y0)/(x1-x0)
     */       
    void defineFunction_l();
    
    /** @brief definition of circles
     *  
     * represent a circle defined by x = a*cos(2*PI*f*t); y = a*sin(2*PI*f*t);
     */        
    void defineFunction_c();
    
    /** @brief definition of sinusoid
     *  
     * represent a sinus defined by y = a*sin(2*PI*f*t + phi) 
     */    
    void defineFunction_s();
    
    /** @brief definition of parabolic
     *  
     * represent a parabolic defined by y = ax^2
     */        
    void defineFunction_p();
    
    /** @brief initialisation
     *  
     * initialize all variables needed for MainWindow
     */         
    void initializeApproxVariables();
 
public slots:
    /** @brief open a file
     *  
     * open a  .svg file 
     */   
    void openFile();
    
    /** @brief close a file
     *  
     * close a opened file .svg
     */   
    void closeFile();
    
    /** @brief fullscreen mode
     *  
     * maximize the MainWindow using F11 key
     */       
    void fullScreen();
    
    /** @brief computation of the approximated trajectory
     *  
     * compute the approximated trajectory if a theorical 
     * trajectory is already loaded
     */       
    void computeTraj();
    
    /** @brief computation of SoftMotion
     *  
     * compute the canonical softMotion trajectory between the
     * two specified initial and final kinematic conditions
     */       
    void computeSoftMotion();
    
    /** @brief generate a data file
     *  
     * export the discretized approximated trajectory into a file
     * with a specified sampling time ( 10 ms by default)
     */       
    void genFileTraj();
    
    /** @brief computation of hausdorff distance
     *  
     * compute the hausdorff distance between the ideal and 
     * approximated trajectory
     */       
    void computeHausdorff();
    
    /** @brief display a traced trajectory
     *  
     * turn on/off the display of a traced trajectory
     */       
    void setDraw();

protected:
    void changeEvent(QEvent *e);

private:
    /** @brief plot of motion law
     *  
     * plot the motion law "SoftMotion"
     *
     * @param &curv : a curve with motion law
     */   
    void plotMotionLaw(Curve &curv);
    
    /** @brief plot of motion profile
     *  
     * plot the ideal trajectory with "SoftMotion" in axis x and y separately
     *
     * @param &curv : a curve with motion law
     */     
    void plotIdealProfile(Curve &curv);
    
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
    void plotResults(Curve &curv2);
    
    /** @brief plot of errors
     *  
     * plot the error of ideal and approximated trajectories
     *
     * @param &curv2 : a curve with motion law
     * @param &error : a vector of errors
     * @param *val_err_max : pointer of the maximum value of error
     */      
    void plotErrors(Curve &curv2, std::vector<double> &error, double *val_err_max);
    
    /** @brief plot of hausdorff distance
     *  
     * plot the hausdorff distance between the ideal and approximated trajectories
     *
     * @param &dis1 : the first vector of hausdorff distance 
     * @param &dis2 : the second vector of hausdorff distance
     * @param sup1 : the maximum value of the first vector hausdorff 
     * @param sup2 : the maximum value of the second vector hausdorff
     */       
    void plotHaus(std::vector<double> &dis1, std::vector<double> &dis2, double sup1, double sup2);


private:
    /* Viewer params */
    int _nbCurve;
    std::string _fileName;
    bool _isFullScreen;

    
};

#endif // MAINWINDOW_H
