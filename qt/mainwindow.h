#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <string>
#include "curve.h"
#include "ui_mainwindow.h"

namespace Ui {
    class MainWindow;
}

class MainWindow :  public QMainWindow, public Ui_MainWindow  {
    Q_OBJECT
    
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void loadSvgFile(std::string str);
    
public slots:
    void openFile();
    void closeFile();
    void fullScreen();
    void computeTraj();
    void computeSoftMotion();
    void test();

protected:
    void changeEvent(QEvent *e);
//     virtual void keyPressEvent(QKeyEvent *e);
//     virtual QString helpString() const;

private:
    void plotMotionLaw(Curve &curv);
    void plotIdealProfile(Curve &curv);
    void plotGraph(QwtPlot *p, double xData[], double yData[], int size, char* xName, char *yName, char *title);
    void plotResults(Curve &curv2);
    void plotErrors(Curve &curv2, std::vector<double> &error, double *val_err_max);
    void plotHaus(std::vector<double> &dis1, std::vector<double> &dis2, double *sup1, double *sup2);
    std::string fileName;
    bool isFullScreen;

private:
    /* Viewer params */
    int nbCurve;


    
};

#endif // MAINWINDOW_H
