#ifndef CURVE_H
#define CURVE_H

#include "../src/softMotionStruct.h"
#include <qglviewer.h>

class Curve : public QGLViewer
{
public :
  Curve();
  Curve(const Curve& curv);
  Curve& operator=(const Curve& curv);
  ~Curve();
  
  void setColor(float f1, float f2, float f3);
  void draw();
  void createPath(std::string file);
  int  nbKeyFrames();
  int  currentKf();
  void setCurrentKf(int v);
  bool isDraw();
  void setIsDraw(bool v);
  double width;
  double height;
  
  qglviewer::ManipulatedFrame** keyFrame_;
  qglviewer::KeyFrameInterpolator kfi_;

  kinPoint errorMax;
  std::vector<kinPoint> discPoint;
  std::vector<SM_CURVE_DATA> traj;//faut tracer
  std::list<Path> path;

protected :


private :
  float _color_f1;
  float _color_f2;
  float _color_f3;
  int _currentKF;
  int _nbKeyFrames;
  bool _isDraw;

};

#endif // CURVE_H