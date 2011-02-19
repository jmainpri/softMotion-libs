#ifndef SM_CURVE_H
#define SM_CURVE_H

#include "softMotionStruct.h"

/** @file Sm_Curve.h
 * @brief This file includes the declaration of class Curve
 */

/** @brief Class of the trajectory traced in Mainwindow
 *
 */

class Sm_Curve
{
public :
/** @brief Constructor
  *  
  * The constructor of class Curve
  */  
  Sm_Curve();
  
/** @brief Constructor
  *  
  * The constructor of class Curve with parameters
  *
  * @param &curv : represent a trajectory 
  */  
  Sm_Curve(const Sm_Curve& curv);
  
  Sm_Curve& operator=(const Sm_Curve& curv);
  
/** @brief Destructor
  *  
  * The destructor of class Curve
  */    
  ~Sm_Curve();
  
  double width;
  double height;

  kinPoint errorMax;
  double errorMaxVal;
  std::vector<kinPoint> discPoint;
  std::vector<SM_CURVE_DATA> traj;
  std::vector<SM_OUTPUT> segTraj;
  std::list<Path> path;
  std::list<SubTraj> trajList;


protected :


private :

};

#endif // SM_CURVE_H
