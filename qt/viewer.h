
#include <qglviewer.h>
#include "curve.h"

class Viewer :  public QGLViewer
{
public :
  Viewer(QWidget *parent = NULL);
//   virtual ~Viewer();

  std::vector<Curve> curve;
 
protected :
    virtual void draw();
    virtual void keyPressEvent(QKeyEvent *e);

private :
  double _gridSize;
  double _gridStep;
  int _currentKF_;
  int _nbCurve;
};
