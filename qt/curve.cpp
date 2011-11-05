#include "curve.h"
#ifdef ENABLE_DISPLAY
#if QT_VERSION >= 0x040000
# include <QKeyEvent>
#endif
#endif
#include <iostream>
#include <fstream>
#include <string>

/** @file curve.cpp
 * @brief This file includes the constructor and functions related for class Curve
 */

#ifdef ENABLE_DISPLAY
using namespace qglviewer;
#endif
using namespace std;

Curve::Curve()
{
  _color_f1 = 1.;
  _color_f2 = 1.;
  _color_f3 = 1.;

  for(int i=0; i<3; i++) {
    errorMax.kc[i].a = 0.0;
    errorMax.kc[i].v = 0.0;
    errorMax.kc[i].x = 0.0;
  }
  errorMax.t = 0.0;
  discPoint.clear();
  traj.clear();

}

Curve::Curve (const Curve& c):
  QGLViewer()
{
#ifdef ENABLE_DISPLAY
  _isDraw = c._isDraw;
  _color_f1 = c._color_f1;
  _color_f2 = c._color_f2;
  _color_f3 = c._color_f3;
#endif
  path = c.path;
  traj = c.traj;
  trajList = c.trajList;
   discPoint = c.discPoint;
 
  errorMax = c.errorMax;
}
Curve::~Curve()
{
#ifdef ENABLE_DISPLAY
  discPoint.clear();
  traj.clear();
  path.clear();
  trajList.clear();
#endif
}

Curve& Curve::operator=(const Curve& curv)
{
  static Curve o;

  o.discPoint = curv.discPoint;

  return o;
}

void Curve::setColor(float f1, float f2, float f3)
{
  _color_f1 = f1;
  _color_f2 = f2;
  _color_f3 = f3;
  return;
}

void Curve::draw()
{
#ifdef ENABLE_DISPLAY
  // Draw interpolated frame
  //glPushMatrix();
  //glMultMatrixd(kfi_.frame()->matrix());
  //drawAxis(0.01f);
  //glPopMatrix();
  
  if(discPoint.size() > 0) {
	glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
	glDisable(GL_LIGHTING);
 
	glPointSize(10.0);
	glColor3f(0.0, 1.0, 0.0);
	glBegin(GL_POINTS);
	for (unsigned int j = 0; j < discPoint.size(); j++) {
	  glVertex3f(discPoint[j].kc[0].x,discPoint[j].kc[1].x,discPoint[j].kc[2].x);
	}
	glEnd();

	glPointSize(20.0);
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_POINTS);
	glVertex3f(errorMax.kc[0].x,errorMax.kc[1].x,errorMax.kc[2].x);
	glEnd();

	glPopAttrib();

  }

  drawAxis(0.05f);
  glColor3f(_color_f1,_color_f2,_color_f3);
  glPushAttrib(GL_LIGHTING_BIT | GL_LINE_BIT);
  glDisable(GL_LIGHTING);

  glLineWidth(4);
  glBegin(GL_LINES);
  for (unsigned int i=1; i< traj.size(); i++) {
       glVertex3f(traj[i-1].Pos[0], traj[i-1].Pos[1], traj[i-1].Pos[2]);
       glVertex3f(traj[i].Pos[0], traj[i].Pos[1], traj[i].Pos[2]);
  }
  glEnd();
  glPopAttrib();

  glColor3f(1.,1.,1.);
#endif
  return;
}

bool Curve::isDraw()
{
  return _isDraw;
}

void Curve::setIsDraw(bool v)
{
  _isDraw = v;
}

