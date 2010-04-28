#include "viewer.h"

#if QT_VERSION >= 0x040000
# include <QKeyEvent>
#endif

#include <stdio.h>

using namespace qglviewer;
using namespace std;

Viewer::Viewer(QWidget* parent) :
  QGLViewer(parent)
{
camera()->setPosition(qglviewer::Vec( 0., 0., 0.5));
}

void Viewer::keyPressEvent(QKeyEvent *e)
{
  switch (e->key())
    {
    case Qt::Key_Left :
      curve[0].setCurrentKf( (curve[0].currentKf()+curve[0].nbKeyFrames()-1) % curve[0].nbKeyFrames() );
      curve[0].setManipulatedFrame(curve[0].keyFrame_[curve[0].currentKf()]);
      updateGL();
      break;
    case Qt::Key_Right :
      curve[0].setCurrentKf( (curve[0].currentKf()+1) % curve[0].nbKeyFrames() );
      curve[0].setManipulatedFrame(curve[0].keyFrame_[curve[0].currentKf()]);
      updateGL();
      break;
    case Qt::Key_Return :
       curve[0].kfi_.toggleInterpolation();
      break;
    case Qt::Key_Plus :
      curve[0].kfi_.setInterpolationSpeed(curve[0].kfi_.interpolationSpeed()+0.25);

      break;
    case Qt::Key_Minus :
      curve[0].kfi_.setInterpolationSpeed(curve[0].kfi_.interpolationSpeed()-0.25);
      break;
    // case Qt::Key_C :
      // kfi_.setClosedPath(!kfi_.closedPath());
      // break;
    case Qt::Key_Space :
        curve[0].setCurrentKf(0);
	
        curve[0].setManipulatedFrame(curve[0].keyFrame_[0]);
	curve[0].kfi_.setInterpolationSpeed(curve[0].nbKeyFrames()/30.0);
	curve[0].kfi_.setLoopInterpolation();
	curve[0].kfi_.startInterpolation();
      break;
    default:
      QGLViewer::keyPressEvent(e);
    }
}

void Viewer::draw()
{
  // Draw interpolated frame
  char text[128];
  _gridSize = 0.1;
  _gridStep = 4;

  drawGrid(0.1,4);
  setTextIsEnabled(true);
  for(double i = -_gridSize; i<=_gridSize;i = i+(_gridSize/(_gridStep /2))){
     sprintf(text,"%.3f",i);
     QGLWidget::renderText(i,0,0, text);
     QGLWidget::renderText(0,i,0, text);
  }

 
  for(unsigned int i=0; i<curve.size(); i++) {
       if(curve[i].isDraw() == true) {




if(i==0) {
  glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
  glDisable(GL_LIGHTING);
 
  glPointSize(5.0);
  glColor3f(1.0, 1.0, 0.0);
  glBegin(GL_POINTS);
  for (unsigned int j = 0; j < curve[i].discPoint.size(); j++) {
    glVertex3f(curve[i].discPoint[j].kc[0].x,curve[i].discPoint[j].kc[1].x,0.);
  }
  glEnd();

  glPointSize(10.0);
  glColor3f(1.0, 0.0, 0.0);
  glBegin(GL_POINTS);
  glVertex3f(curve[0].errorMax.kc[0].x,curve[0].errorMax.kc[1].x,0.);
  glEnd();

  glPopAttrib();
// curve[i].setColor(1,1,1);

}
if(i==1) {
curve[i].setColor(1,0,0);
}
	 
	 curve[i].draw();
       }
  }
}
