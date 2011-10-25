/*
  #
  # Copyright (c) 2010 LAAS/CNRS
  # All rights reserved.
  #
  # Permission to use, copy, modify, and distribute this software for any purpose
  # with or without   fee is hereby granted, provided   that the above  copyright
  # notice and this permission notice appear in all copies.
  #
  # THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
  # REGARD TO THIS  SOFTWARE INCLUDING ALL  IMPLIED WARRANTIES OF MERCHANTABILITY
  # AND FITNESS. IN NO EVENT SHALL THE AUTHOR  BE LIABLE FOR ANY SPECIAL, DIRECT,
  # INDIRECT, OR CONSEQUENTIAL DAMAGES OR  ANY DAMAGES WHATSOEVER RESULTING  FROM
  # LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
  # OTHER TORTIOUS ACTION,   ARISING OUT OF OR IN    CONNECTION WITH THE USE   OR
  # PERFORMANCE OF THIS SOFTWARE.
  #
  #                                            Xavier BROQUERE on Fri Feb 26 2010
*/
/*--------------------------------------------------------------------
  Fichier              : softMotionStruct.h
  Fonction             : Structures definition for 3D Motion Trajectory Planning
  Date de creation     : Mai 2008
  Date de modification : Mai 2008
  Nb de lignes         :

  Auteur               : Xavier BROQUERE

  ----------------------------------------------------------------------*/

#ifndef SM_TRAJ_H
#define SM_TRAJ_H

#include "softMotionConst.h"
#include "softMotionStruct.h"
#include "softMotionStructGenom.h"

class SM_TRAJ {
 private:
  int trajId;
  double timePreserved;
  double duration;
  /* time scale vector needed to slow the motion law at the execution 
     this vector is computed by  computeMaxTimeScaleVector Voir these chapitre 5 (fin)*/
  std::vector<double> tsVec;
  std::vector<double> duration_axis;

 public:
  std::vector<double> qStart;
  std::vector<double> qGoal;
  std::vector< std::vector<SM_SEG> > traj;


  enum SM_TRAJ_MODE {
    SM_SYNCHRONIZED,
    SM_INDEPENDANT
  };


 public:
  void sm_traj();
  void sm_traj(const SM_TRAJ &traj);
  
  void clear();
  /**********************
   * Setters / Getters
   **********************/
  std::vector<double> getTsVec() {
    return tsVec;
  }
  int getTrajId();
  void setTrajId(int id);
  int getTimePreserved();
  void setTimePreserved(int t);
  double getDuration();
  void setQStart(std::vector<double> &qs);
  void setQGoal(std::vector<double> &qg);
  void setTraj(std::vector<std::vector<SM_SEG> > &t);

  int getMotionCond(double time, std::vector<SM_COND> & cond);
  int computeTimeOnTraj();
  int updateIC();
  void print();
  void printQStart();
  void printQGoal();
  void resize(int size);
  int append(SM_TRAJ_STR inTraj);
  int save(char *name);
  int load(char *name, int (*fct(void)));
  int convertToSM_TRAJ_STR(SM_TRAJ_STR *smTraj);
  int importFromSM_TRAJ_STR(const SM_TRAJ_STR *smTraj);
  
  int importFromSM_OUTPUT(int trajId, double sampling, std::vector<SM_OUTPUT>  &trajIn);
  int convertToSM_OUTPUT(int trajId, double sampling, std::vector<SM_OUTPUT> &trajIn);
  int approximateSVGFile( double jmax,  double amax,  double vmax,  double SampTime, double ErrMax, char *fileName);

  int approximate(std::vector< std::vector<SM_COND> > &trajIn, double timeStep, double errorPosMax,double errorVelMax, int id, bool flag);
 
  int computeMaxTimeScaleVector(std::vector<double> & maxVel, double tic, SM_LIMITS timeLimits);
  int computeMaxTimeScaleVectorTest(std::vector<double> & maxVel, double tic, SM_LIMITS timeLimits);
  int extract(double t1, double t2, SM_TRAJ &trajIn);

  /* Obsolete use computeTraj instead */
  int computeOneDimTraj(SM_COND IC, SM_COND FC, SM_LIMITS limits);

  /* compute a multidimensional trajectory */
  int computeTraj(std::vector<SM_COND> IC, std::vector<SM_COND> FC, std::vector<SM_LIMITS> limits, SM_TRAJ_MODE mode);
  int plot(int i);
  int plot();

 private:
   
  int getSegmentIndex(double t1);
  void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters);
  std::vector<double> parseFrame(std::string& line);
  int fillFromMotionArr(std::vector<SM_MOTION_AXIS> &motion_arr);
  int computeUnsynchronizedMotion(std::vector<SM_MOTION_AXIS> &motion_arr);
  int synchronizeMotion(std::vector<SM_MOTION_AXIS>  &motion_arr);
};

#endif
