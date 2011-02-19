#include "Sm_Curve.h"

#include <iostream>
#include <fstream>
#include <string>

/** @file Sm_Curve.cpp
 * @brief This file includes the constructor and functions related for class Sm_Curve
 */


using namespace std;

Sm_Curve::Sm_Curve()
{
  //for(int i=0; i<3; i++) {
  //  errorMax.kc[i].a = 0.0;
  //  errorMax.kc[i].v = 0.0;
  //  errorMax.kc[i].x = 0.0;
  //}
  errorMax.t = 0.0;
  errorMaxVal = 0.0;
  discPoint.clear();
  traj.clear();
  
}

Sm_Curve::Sm_Curve (const Sm_Curve& c)
{
  path = c.path;
  traj = c.traj;
  trajList = c.trajList;
  discPoint = c.discPoint; 
  errorMax = c.errorMax;
  errorMaxVal = c.errorMaxVal;
}
Sm_Curve::~Sm_Curve()
{
  discPoint.clear();
  traj.clear();
  path.clear();
  trajList.clear();
}

Sm_Curve& Sm_Curve::operator=(const Sm_Curve& curv)
{
  static Sm_Curve o;
  o.discPoint = curv.discPoint;
  return o;
}

