#include "viewer.h"

#if QT_VERSION >= 0x040000
# include <QKeyEvent>
#endif

#include <stdio.h>
#include <GL/gl.h>

/** @file viewer.cpp
 * @brief This file includes the constructor and functions related for class Viewer
 */

using namespace qglviewer;
using namespace std;

Viewer::Viewer(QWidget* parent) :
  QGLViewer(parent)
{
camera()->setPosition(qglviewer::Vec( 0., 0., 0.5));
}

void Viewer::keyPressEvent(QKeyEvent *e)
{
//  switch (e->key())
//    {
//    case Qt::Key_Left :
//      curve[0].setCurrentKf( (curve[0].currentKf()+curve[0].nbKeyFrames()-1) % curve[0].nbKeyFrames() );
//      curve[0].setManipulatedFrame(curve[0].keyFrame_[curve[0].currentKf()]);
//      updateGL();
//      break;
//    case Qt::Key_Right :
//      curve[0].setCurrentKf( (curve[0].currentKf()+1) % curve[0].nbKeyFrames() );
//      curve[0].setManipulatedFrame(curve[0].keyFrame_[curve[0].currentKf()]);
//      updateGL();
//      break;
//    case Qt::Key_Return :
//       curve[0].kfi_.toggleInterpolation();
//      break;
//    case Qt::Key_Plus :
//      curve[0].kfi_.setInterpolationSpeed(curve[0].kfi_.interpolationSpeed()+0.25);
//
//      break;
//    case Qt::Key_Minus :
//      curve[0].kfi_.setInterpolationSpeed(curve[0].kfi_.interpolationSpeed()-0.25);
//      break;
//    // case Qt::Key_C :
//      // kfi_.setClosedPath(!kfi_.closedPath());
//      // break;
//    case Qt::Key_Space :
//        curve[0].setCurrentKf(0);
//
//        curve[0].setManipulatedFrame(curve[0].keyFrame_[0]);
//	curve[0].kfi_.setInterpolationSpeed(curve[0].nbKeyFrames()/30.0);
//	curve[0].kfi_.setLoopInterpolation();
//	curve[0].kfi_.startInterpolation();
//      break;
//    default:
//      QGLViewer::keyPressEvent(e);
//    }
}

void Viewer::draw()
{
  // Draw interpolated frame
  char text[128];
  _gridSize = 0.1;
  _gridStep = 4;





setBackgroundColor      (    QColor(Qt::white)      );
setForegroundColor      (    QColor(Qt::black)      );

glColor3f(0,0,0);
  drawGrid(0.2,4);

 setSceneBoundingBox    ( Vec(-10,-10,-10),Vec(10,10,10));

  setTextIsEnabled(true);
  for(double i = -_gridSize*(_gridStep /2); i<=_gridSize*(_gridStep /2);i = i+_gridSize){
    sprintf(text,"%.3f",i);
    QGLWidget::renderText(i,0,0, text);
    QGLWidget::renderText(0,i,0, text);
  }

  QGLWidget::renderText(_gridSize*(_gridStep /2)+_gridSize/10.0,-_gridSize/8.0,0, "X (m)");
  QGLWidget::renderText(-_gridSize/3.0,_gridSize*(_gridStep /2)+_gridSize/10.0,0, "Y (m)");
  for(unsigned int i=0; i<curve.size(); i++) {
    if(curve[i].isDraw() == true) {

      if(i>=1) {

	curve[i].setColor(1,0,0);
      } else {
curve[i].setColor(0,0,1 );
      }

      curve[i].draw();
    }
  }
}
