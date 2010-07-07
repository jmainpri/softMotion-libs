
#include <qglviewer.h>
#include "curve.h"

/** @file viewer.h
 * @brief This file includes the declaration of class Viewer
 */

/** @brief Class of the viewer for the Mainwindow
 *
 */

class Viewer :  public QGLViewer
{
public :
/** @brief Constructor
  *  
  * The constructor of class Viewer
  *
  * @param *parent : NULL 
  */   
  Viewer(QWidget *parent = NULL);
//   virtual ~Viewer();

  std::vector<Curve>  curve;
 
protected :
    virtual void draw();
    virtual void keyPressEvent(QKeyEvent *e);

private :

  double _gridSize;
  double _gridStep;
  int _currentKF_;
  int _nbCurve;
};
