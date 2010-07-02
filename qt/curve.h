#ifndef CURVE_H
#define CURVE_H

#include "../src/softMotionStruct.h"
#include <qglviewer.h>

/** @file curve.h
 * @brief This file includes the declaration of class Curve
 */

/** @brief This class represents the trajectory traced in Mainwindow
 *
 */

class Curve : public QGLViewer
{
public :
/** @brief Constructor
  *  
  * The constructor of class Curve
  */  
  Curve();
  
/** @brief Constructor
  *  
  * The constructor of class Curve with parameters
  *
  * @param &curv : represent a trajectory 
  */  
  Curve(const Curve& curv);
  
  Curve& operator=(const Curve& curv);
  
/** @brief Destructor
  *  
  * The destructor of class Curve
  */    
  ~Curve();
  
/** @brief set colors
  *  
  * set different colors for the trajectoies
  *
  * @param f1 : represent a color
  * @param f2 : represent a color
  * @param f3 : represent a color
  */       
  void setColor(float f1, float f2, float f3);
  
/** @brief draw the trajectoies
  *  
  * draw the trajectoies
  */      
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
  std::vector<SM_CURVE_DATA> traj;
  std::list<Path> path;
  std::list<SubTraj> trajList;


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
