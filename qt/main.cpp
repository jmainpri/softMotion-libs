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
#endif
}
