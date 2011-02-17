#ifndef CURVE_H
#define CURVE_H

#include "../src/softMotionStruct.h"
#ifdef ENABLE_DISPLAY
#include <qglviewer.h>
#endif

/** @file curve.h
 * @brief This file includes the declaration of class Curve
 */

/** @brief Class of the trajectory traced in Mainwindow
 *
 */

class Curve
#ifdef ENABLE_DISPLAY
 : public QGLViewer
#endif
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
  bool isDraw();
  void setIsDraw(bool v);
  double width;
  double height;

  kinPoint3 errorMax;
  std::vector<kinPoint3> discPoint;
  std::vector<SM_CURVE_DATA3> traj;
  std::vector<SM_OUTPUT> segTraj;
  std::list<Path> path;
  std::list<SubTraj3> trajList;


protected :


private :
  float _color_f1;
  float _color_f2;
  float _color_f3;
  bool _isDraw;

};

#endif // CURVE_H
