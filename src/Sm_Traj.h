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

 public:
  std::vector<double> qStart;
  std::vector<double> qGoal;
  std::vector< std::vector<SM_SEG> > traj;


 public:
  void sm_traj();
  void sm_traj(const SM_TRAJ &traj);
  
  void clear();
  /*
   * Setters / Getters
   */
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
  void resize(int size);
  int append(SM_TRAJ_STR inTraj);
  int save(char *name);
  int load(char *name, int (*fct(void)));
  int convertToSM_TRAJ_STR(SM_TRAJ_STR *smTraj);
  int importFromSM_TRAJ_STR(const SM_TRAJ_STR *smTraj);
  
  int importFromSM_OUTPUT(int trajId, std::vector<SM_OUTPUT>  &trajIn);

  int approximateSVGFile( double jmax,  double amax,  double vmax,  double SampTime, double ErrMax, char *fileName);

  int approximateAVX(std::vector< std::vector<SM_COND> > &trajIn, std::vector<double> vmax, std::vector<double> amax, double timeStep, double errorMax, int id);

 private:
  void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters);
  std::vector<double> parseFrame(std::string& line);
 
};

#endif
