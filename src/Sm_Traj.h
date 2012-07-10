/*--------------------------------------------------------------------
  Fichier              : softMotionStruct.h
  Fonction             : Class definition for Trajectory Generation
  Date de creation     : Mai 2008
  Date de modification : Mai 2012
  Nb de lignes         :

  Auteur               : Xavier BROQUERE 
  Web                  : www.broquere.fr

  ----------------------------------------------------------------------*/


/** @mainpage Soft Motion Trajectory Planner
 *
 * @section Copyright
 *
 * Copyright (c) 2012 LAAS/CNRS
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software for any purpose
 * with or without   fee is hereby granted, provided   that the above  copyright
 * notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
 * REGARD TO THIS  SOFTWARE INCLUDING ALL  IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS. IN NO EVENT SHALL THE AUTHOR  BE LIABLE FOR ANY SPECIAL, DIRECT,
 * INDIRECT, OR CONSEQUENTIAL DAMAGES OR  ANY DAMAGES WHATSOEVER RESULTING  FROM
 * LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
 * OTHER TORTIOUS ACTION,   ARISING OUT OF OR IN    CONNECTION WITH THE USE   OR
 * PERFORMANCE OF THIS SOFTWARE.
 *
 *                                           Xavier BROQUERE on Feb 01 2012
 *
 * @section Author
 * Dr Xavier BROQUERE
 * xavier@broquere.fr
 * www.broquere.fr
 * 
 * @section Introduction
 *
 * The soft motion trajectory generation aims to compute jerk bounded trajectories
 *
 *
 * Software description here
 */

#ifndef SM_TRAJ_H
#define SM_TRAJ_H

#include "softMotionConst.h"
#include "softMotionStruct.h"
#include "softMotionStructGenom.h"

class SM_TRAJ {

 public:

  std::vector<double> qStart;
  std::vector<double> qGoal;
  std::vector<double> jmax;
  std::vector<double> amax;
  std::vector<double> vmax;
  std::vector< std::vector<SM_SEG> > traj;

  // virtualTime = 0.0 at beginning of the Traj, and 1.0 at the end;
  // one virtualTime for each SM_SEG
  std::vector< std::vector<double> > virtualTimeOnTraj;

  enum SM_TRAJ_MODE {
    SM_SYNCHRONIZED, // not implemented
    SM_INDEPENDANT,
    SM_3SEGMENT
  };

  enum SM_TRAJ_TYPE {
    SM_STOP_AT_VIA_POINT,
    SM_SMOOTH_AT_VIA_POINT // not implemented
  };

  /*! Constructor
   */
  SM_TRAJ();
  /*! Copy construtor
   */
  SM_TRAJ(const SM_TRAJ &traj);
  
  /*! clear the trajectory
   */
  void clear();
  
  /*! clear the trajectory and resize it to the specified size
   */ 
  void resize(int size);

  /*! append the trajectory stored in a SM_TRAJ_STR to the current traj
   *  it appends only the segments and do not verify the absolute position (TODO...)
   */
  int append(SM_TRAJ_STR inTraj);
  
  /*! append another traj to the current traj
   *  it appends only the segments and do not verify the absolute position (TODO...)
   */
  int append(SM_TRAJ &smTraj);
  
  /*! clear the current trajectory and extract a subtrajectory from an another one
   */
  int extract(double t1, double t2, SM_TRAJ &trajIn);

  /*! save a trajectory into a .traj file
   * @see load(char* name)
   */
  int save(char *name);
  
  /*! load a trajectory from a .traj file
   * @see save(char* name)
   */
  int load(char *name);
  int load(char *name, int(*fct(void)) );
  /*! convert the SM_TRAJ to a static struture SM_TRAJ_STR with a fixed maximun size
   * @see importFromSM_TRAJ_STR(const SM_TRAJ_STR *smTraj)
   */
  int convertToSM_TRAJ_STR(SM_TRAJ_STR *smTraj);
  
  /*! clear the SM_TRAJ and fill it from a static struture SM_TRAJ_STR
   * @see convertToSM_TRAJ_STR(SM_TRAJ_STR *smTraj)
   */
  int importFromSM_TRAJ_STR(const SM_TRAJ_STR *smTraj);
 
  /*! compute one dimensional trajectory
   * DEPRECATED 
   * @see computeTraj(std::vector<SM_COND> IC, std::vector<SM_COND> FC, std::vector<SM_LIMITS> limits, SM_TRAJ_MODE mode)
   * @see computeTraj(std::vector< std::vector<double> > pos, std::vector<SM_LIMITS> limits, SM_TRAJ_TYPE mode)
   */
  int computeOneDimTraj(SM_COND IC, SM_COND FC, SM_LIMITS limits);

  /*! compute a multidimensional trajectory between two kinematic states
   *  @param IC the initial conditions
   *  @param FC the final conditions
   *  @param limits the kinematic constraints
   *  @param mode 
   *      \li SM_SYNCHRONIZED currently not implemented
   *      \li SM_INDEPENDANT the duration of trajectories of each axis are not synchronized
   */
  int computeTraj(std::vector<SM_COND> IC, std::vector<SM_COND> FC, std::vector<SM_LIMITS> limits, SM_TRAJ_MODE mode);
  int computeTraj(std::vector<SM_COND> IC, std::vector<SM_COND> FC, std::vector<SM_LIMITS> limits, SM_TRAJ_MODE mode, std::vector<double> imposedDuration);

  /*! compute a multidimensional trajectory given n via points
   *  @param pos a vector of N dimensional vectors representing the via points in the N dimensional space
   *  @param limits a N dimensional vector of kinematic constraints
   *  @param mode 
   *       \li SM_STOP_AT_VIA_POINT the trajectory pass at each via point with NULL velocity and acceleration 
   *       \li SM_SMOOTH_AT_VIA_POINT the trajectory does not pass at the via points but near and without stopping
   */
  int computeTraj(std::vector< std::vector<double> > pos, std::vector<SM_LIMITS> limits, SM_TRAJ_TYPE mode);

  /** approximate a discrete trajectory
   * @param trajIn the discrete trajectory in a array of SM_COND
   * @param timeStep the sampling time of the discretized trajIn
   * @param errorPosMax the desired maximun error in position for each axis
   * @param errorVelMax the desired maximun error in velocity for each axis
   * @param id the id that you want for the output traj
   * @param flag enable exporting the approximated traj in a file (advice: put true)
   * @return 1: error 0: OK
   */
  int approximate(std::vector< std::vector<SM_COND> > &trajIn, double timeStep, double errorPosMax,double errorVelMax, int id, bool flag);
  
  /** compute a trajectory from an svg file and kinematic constraints on the motion law
   * @param jmax the maximum jerk of the motion law
   * @param amax the maximum acceleration of the motion law
   * @param vmax the maximum velocity of the motion law
   * @param sampTime the sampling time to compute the approximation error
   * @param errMax the maximum trajectory error allowed
   * @param fileName the svg file
   * @return 1: error 0: OK
   */
  int approximateSVGFile( double jmax,  double amax,  double vmax,  double sampTime, double errMax, char *fileName);

  /*! once the trajectory is computed, getMotionCond() gets the kinematic state of each axis
   *  @param time the instant wanted
   *  @param cond the kinematic state filled and resized the number of axes
   */
  int getMotionCond(double time, std::vector<SM_COND> & cond);
  int getMotionCondVT(double vTime, std::vector<SM_COND> &cond);

  double getVirtualTime(double time);


  /*! plot the evolution of the position, velocity and acceleration of the specified axis using gnuplot
   * @param i the axis id
   */
  int plot(int i);
  
  /*! plot the evolution of the position of all axes using gnuplot
   */
  int plot();

  /*! print the trajectory into the console
   */
  void print();
  
  /*! print the vector of intial positions in the console
   */
  void printQStart();
  
  /*! print the vector of final positions in the console
   */
  void printQGoal();
  
  /*! check if the trajectory is valid considering the kinematic constraints, the initial and final conditions
   */
  int checkTrajBounds(double time_step, std::vector<SM_COND> IC, std::vector<SM_COND> FC);

  /*! compute and update the initial conditions of each segment for each axis for the all 
   *  trajectory from the initial conditions, the duration and the jerk values of each segment 
   */
  int updateIC();

  /*! compute and update the position in time of each segment for each axis
   *  @see SM_SEG
   */
  int computeTimeOnTraj();

  /*! compute a one dimensional vector with a size as getDuration()/tic
   *  the goal of this vector is to compute the maximum timeScale allowed to satisfy the maximun velocity for each axis (@param maxVel)
   *  the vector is a smoothed function with the parameter @param timeLimits
   *  this vector is then used a runtime to slow down the motion law to not overshoot the maximum allowd velocities
   *  @param maxVel the vector of maximum velocities for each axis
   *  @param tic the sampling time if the vector (must be the same as the controller one)
   *  @param timeLimits kinematic constraints of the motion law
   */
  int computeMaxTimeScaleVector(std::vector<double> & maxVel, double tic, SM_LIMITS timeLimits);
  
  /*! get the trajId member
   * trajId the user specified trajectory index
   * this index can be changed at any time @see setTrajId(int id)
   */
  int getTrajId();
  
  /*! set the trajId member
   */
  void setTrajId(int id);
  
  /*! get the timePreserved member
   * this member is used as a reference time to switch between trajectory
   * @see setTimePreserved(int t)
   */
  int getTimePreserved();
  
  /*! set the timePreserved value
   *  @see getTimePreserved()
   */
  void setTimePreserved(int t);
  
  /*! get the duration of the trajectory
   */
  double getDuration();


//get max values in the vector

  std::vector<double> getVmax() const;

  std::vector<double> getAmax() const;

  std::vector<double> getJmax() const;


  /*!  set the vector of initial positions (only informative data)
   */
  void setQStart(std::vector<double> &qs);
  
  /*! set the vector of final positions (only informative data)
   */
  void setQGoal(std::vector<double> &qg);
  
  /*! set manually the trajectory with an array of SM_SEG
   */
  void setTraj(std::vector<std::vector<SM_SEG> > &t);

  /*! get the computed timeScale vector
   * @see computeMaxTimeScaleVector(std::vector<double> & maxVel, double tic, SM_LIMITS timeLimits)
   */
  std::vector<double> getTsVec() {
    return tsVec;
  }

   int mergetwotrajectories( SM_TRAJ &Trajfinal, SM_TRAJ &Trajinitial);


  /*!
   */
  int computeMaxTimeScaleVectorTest(std::vector<double> & maxVel, double tic, SM_LIMITS timeLimits);




  int importFromSM_OUTPUT(int trajId, double sampling, std::vector<SM_OUTPUT>  &trajIn);
  int convertToSM_OUTPUT(int trajId, double sampling, std::vector<SM_OUTPUT> &trajIn);





 private:
  /*! the user specified trajectory index
   */
  int trajId;

  /*! the user specified time preserved instant. Used for replanning and switching between trajectories
   */
  double timePreserved;
  
  /*! the duration of the trajectory
   */
  double duration;
  
  /* time scale vector needed to slow the motion law at the execution 
     this vector is computed by  computeMaxTimeScaleVector Voir these chapitre 5 (fin)*/
  std::vector<double> tsVec;
  std::vector<double> duration_axis;

  int getSegmentIndex(double t1);
  void tokenize(const std::string& str, std::vector<std::string>& tokens, const std::string& delimiters);
  std::vector<double> parseFrame(std::string& line);
  int fillFromMotionArr(std::vector<SM_MOTION_AXIS> &motion_arr);
  int computeUnsynchronizedMotion(std::vector<SM_MOTION_AXIS> &motion_arr);
  int computeSynchronizedMotion(std::vector<SM_MOTION_AXIS> &motion_arr);
  int ThreeSegSynchronizedMotion(std::vector<SM_MOTION_AXIS>  &motion_arr);
  //int synchronizedMotion(std::vector<SM_MOTION_AXIS> &motion_arr); 
  //to implement
};

#endif
