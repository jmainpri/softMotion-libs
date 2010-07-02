#include <QtGui/QApplication>
#include "mainwindow.h"

/** @file main.cpp
 * @brief This is the main function
 */

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.showMaximized();
    return a.exec();
}
