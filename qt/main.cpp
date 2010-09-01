#include <QtGui/QApplication>
#include "QSoftMotionPlanner.h"
#include <iostream>
//#include "mainwindow_nw.h"

using namespace std;
/** @file main.cpp
 * @brief This is the main function
 */

int main(int argc, char *argv[])
{
#ifdef ENABLE_DISPLAY
  QApplication a(argc, argv);
#endif

  QSoftMotionPlanner w;

#ifndef ENABLE_DISPLAY
  w.setDisplay(false);
#endif

#ifdef ENABLE_DISPLAY
  w.setDisplay(true);
  w.showMaximized();
  return a.exec();  
#else
//   w.approximate(double jmax,double amax,double vmax,double SampTime,double ErrMax, int ExpTime, FILE* fileptr, std::vector<SM_OUTPUT> result);
  // w.approximate(jmax, amax, vmax, SampTime, ErrMax, ExpTime, vector< pose >, vector);
  return 0
#endif
}
