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
  -- C N R S --
  Laboratoire d'automatique et d'analyse des systemes
  Groupe Robotique et Intelligenxce Artificielle
  7 Avenue du colonel Roche
  31 077 Toulouse Cedex

  Fichier              : softMotion.c
  Fonctions            : Define and Adjust Time Profile of Jerk
  Date de creation     : June 2007
  Date de modification : June 2007
  Auteur               : Xavier BROQUERE

  ----------------------------------------------------------------------*/

#include "softMotionStruct.h"
#include "softMotion.h"

#include "matrix.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <list>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include "time_proto.h"

#include <vector>

#include <time.h>
#include <stdlib.h>
#ifdef __APPLE__
#include <cstdlib>
#define uint unsigned int
#endif

#include "Sm_Traj.h"
#include "Sm_Curve.h"
#include "Sm_Approx.h"
//#include "QSoftMotionPlanner.h"

#include "gnuplot_i.hpp"


using namespace std;

void SM_TRAJ::sm_traj()
{
  this->clear();
  return;
}

void SM_TRAJ::sm_traj(const SM_TRAJ &traj)
{
  this->trajId = traj.trajId;
  this->timePreserved = traj.timePreserved;
  this->duration = traj.duration;
  this->qStart = traj.qStart;
  this->qGoal = traj.qGoal;
  this->jmax = traj.jmax;
  this->amax = traj.amax;
  this->vmax = traj.vmax;
  this->traj = traj.traj;
  return;
}

void SM_TRAJ::clear() 
{
  qStart.clear();
  qGoal.clear();
  jmax.clear();
  amax.clear();
  vmax.clear();
  traj.clear();
  tsVec.clear();
  duration = 0.0;
  timePreserved = 0.0;
  trajId = 0.0;
  return;
}

int  SM_TRAJ::getTrajId()
{
  return this->trajId;
}

void  SM_TRAJ::setTrajId(int id)
{
  this->trajId = id;
  return;
}

int  SM_TRAJ::getTimePreserved()
{
  return this->timePreserved;
}

void  SM_TRAJ::setTimePreserved(int t)
{
  this->timePreserved = t;
  return;
}

double  SM_TRAJ::getDuration()
{
  return this->duration;
}

void  SM_TRAJ::setQStart(std::vector<double> &qs)
{
  this->qStart = qs;
  return;
}

void  SM_TRAJ::setQGoal(std::vector<double> &qg)
{
  this->qGoal = qg;
  return;
}



double SM_TRAJ::getVmax() const {
   double Vmax = 0.0;
   for(int i=0; i != 6; ++i){
       if(ABS(this->vmax[i]) > Vmax) {
           Vmax = ABS(this->vmax[i]);
       }
    }
    return Vmax;
}

double SM_TRAJ::getAmax() const {
   double Amax = 0.0;
   for(int i=0; i != 6; ++i){
       if(ABS(this->amax[i]) > Amax) {
           Amax = ABS(this->amax[i]);
       }
    }
    return Amax;
}

double SM_TRAJ::getJmax() const {
   double Jmax = 0.0;
   for(int i=0; i != 6; ++i){
       if(ABS(this->jmax[i]) >= Jmax) {
           Jmax = ABS(this->jmax[i]);
       }
    }
    return Jmax;
}




void SM_TRAJ::setTraj(std::vector<std::vector<SM_SEG> > &t)
{
  this->traj = t;
  return;
}

void SM_TRAJ::resize(int size) 
{
  this->clear();
  qStart.resize(size);
  qGoal.resize(size);
  jmax.resize(size);
  amax.resize(size);
  vmax.resize(size);
  traj.resize(size);
  return;
}

int SM_TRAJ::getMotionCond(double time,std::vector<SM_COND> & cond)
{

  SM_COND IC;
  //  SM_STATUS resp;

  //   std::vector<double> t(1);
  //   std::vector<double> a(1);
  //   std::vector<double> v(1);
  //   std::vector<double> x(1);

  cond.clear();
  double dt = 0;
  SM_COND ICl;
  int idSeg = 0;
  double jerk =0;

  //this->computeTimeOnTraj();
  
  for(unsigned int axis=0;  axis< traj.size(); axis++) {

    // Find segment Index
    idSeg = (traj[axis].size()) -1;
    while (time < traj[axis][idSeg].timeOnTraj) { idSeg = idSeg - 1;}
    if(idSeg <0) {
      printf("ERROR Big up, not possible!!\n");
    } 

    dt = time - traj[axis][idSeg].timeOnTraj;

    if(idSeg ==  ((int)traj[axis].size()) -1) {
      if( dt >   traj[axis][idSeg].time) {
	dt =  traj[axis][idSeg].time;
      }
    }

    ICl.a = traj[axis][idSeg].IC.a;
    ICl.v = traj[axis][idSeg].IC.v;
    ICl.x = traj[axis][idSeg].IC.x;
    jerk =  traj[axis][idSeg].jerk;
   
    IC.a =  jerk * dt  + ICl.a;
    IC.v =  jerk * pow(dt,2.0) / 2.0 + ICl.a * dt   + ICl.v;
    IC.x =  jerk * pow(dt,3.0) / 6.0 + ICl.a * pow(dt,2.0) / 2.0 + ICl.v * dt  + ICl.x;
     
    if(axis ==1) {
      //printf("nbseg %d time %f idSeg %d dt %f x %f     X0= %f IC.x= %f\n",traj[axis].size(), time, idSeg, dt, (double)IC.x,(double)traj[axis][0].IC.x, (double)ICl.x );
    }
    cond.push_back(IC);
  }
  
  //      t[0] = time;
  //      resp = sm_AVX_TimeVar(traj[i], t, a, v, x);
  //      IC.a = a[0];
  //      IC.v = v[0];
  //      IC.x = x[0];
  //      cond.push_back(IC);
  
  return 0;
}

//int SM_TRAJ::extractTraj(double time1, double time2, ){
//
//  return 0;
//}
//
int SM_TRAJ::computeTimeOnTraj()
{
  std::vector<double> duration_axis;
  duration_axis.clear();
  duration_axis.resize(traj.size());
  this->duration = 0.0;

  for(unsigned int i=0;  i< traj.size(); i++) {
    for (unsigned int j = 0; j < traj[i].size(); j++){
      if (j == 0) traj[i][j].timeOnTraj = 0.0;
      else {
	traj[i][j].timeOnTraj = 0.0;
	for (unsigned int k = 0; k < j; k ++){
	  traj[i][j].timeOnTraj = traj[i][j].timeOnTraj + traj[i][k].time;
	}
      }
    }
    if(traj[i].size() > 0.0) {
      duration_axis[i] = traj[i][traj[i].size()-1].timeOnTraj + traj[i][traj[i].size()-1].time;
    } else {
      duration_axis[i] = 0.0;
    }
    if(duration_axis[i] > this->duration) {
      this->duration = duration_axis[i];
    }
  }
  updateIC();
  return 0;
}

int SM_TRAJ::checkTrajBounds(double time_step, std::vector<SM_COND> IC, std::vector<SM_COND> FC)
{
  std::vector<SM_COND> cond;
  std::vector<SM_COND> condic;
  double EPS04 = 0.0001;
this->getMotionCond(0.0, condic);
  for(double t=0; t<this->getDuration(); t = t + time_step) {
    cond.clear();
    this->getMotionCond(t, cond);
    for(unsigned int a=0; a<cond.size(); ++a) {
      if(fabs(cond.at(a).a) > this->amax.at(a)) {
	if(fabs(cond.at(a).a) - this->amax.at(a) > EPS04)
	  printf("Acceleration overshoot on axis %i at time %f acc = %f IC(a:%f v:%f x:%f) FC(a:%f v:%f x:%f)\n",a , t, cond.at(a).a,  IC.at(a).a,  IC.at(a).v, IC.at(a).x,   FC.at(a).a,  FC.at(a).v, FC.at(a).x);
      }
      if(fabs(cond.at(a).v) > this->vmax.at(a)) {
	if(fabs(cond.at(a).v) - this->vmax.at(a) > EPS04)
	printf("Velocity overshoot on axis %i at time %f vel = %f  IC(a:%f v:%f x:%f) FC(a:%f v:%f x:%f)\n",a , t, cond.at(a).v,   IC.at(a).a,  IC.at(a).v, IC.at(a).x,   FC.at(a).a,  FC.at(a).v, FC.at(a).x);
      }
      //printf("t %f \n",t);
    }
  }
  return 0;
}

int SM_TRAJ::updateIC()
{

  //  printf("updateIC\n");
  
  //  SM_STATUS resp;
  std::vector<double> t(1);
  std::vector<double> a(1);
  std::vector<double> v(1);
  std::vector<double> x(1);

  SM_COND ICl;
  double time = 0.0;
  double jerk = 0.0;

  // printf("number of segment in the trajectory %d\n",(int)traj[0].size() );
  for(unsigned int axis=0;  axis< traj.size(); axis++) {

    for (unsigned int s = 1; s < (traj[axis].size()) ; s++) {
      ICl.a = traj[axis][s-1].IC.a;
      ICl.v = traj[axis][s-1].IC.v;
      ICl.x = traj[axis][s-1].IC.x;
      time =  traj[axis][s-1].time;
      jerk =  traj[axis][s-1].jerk;
      traj[axis][s].IC.a =  jerk * time  + ICl.a;
      traj[axis][s].IC.v =  jerk * pow(time,2.0) / 2.0 + ICl.a * time   + ICl.v;
      traj[axis][s].IC.x =  jerk * pow(time,3.0) / 6.0 + ICl.a * pow(time,2.0) / 2.0 + ICl.v * time  + ICl.x;

      if(axis == 1) {

	//printf("seg %d IC.a %f IC.v %f IC.x %f \n",s ,traj[axis][s].IC.a,traj[axis][s].IC.v,traj[axis][s].IC.x);
      }
    }
  }
   
  //     for (unsigned int s = 0; s < (traj[axis].size()) ; s++) {
  //       t[0] = traj[axis][s].timeOnTraj;
  //       
  //       resp = sm_AVX_TimeVar(traj[axis], t, a, v, x);
  //       traj[axis][s].IC.a = a[0];
  //       traj[axis][s].IC.v = v[0];
  //       traj[axis][s].IC.x = x[0];     
  //       
  //       if (resp != SM_OK) {
  // 	printf("ERROR: Q interpolation failed (sm_AVX_TimeVar funcion)\n");
  // 	return 1;
  //       }
    
  
  return 0;
}


void SM_TRAJ::print() 
{
  //this->computeTimeOnTraj();
  cout.precision(4);
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl; 
  cout<<  "             TRAJECTORY               " << std::endl;  
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl; 
  cout << " Number of joints: " << traj.size() <<  std::endl;
  cout << " : " << trajId <<  std::endl;
  cout << " timePreserved: " << timePreserved <<  std::endl; 
  cout << " qStart: " <<  std::endl; 
  for(unsigned int j = 0; j < qStart.size(); j++){
    cout << std::fixed << " (" << j << "){" << qStart[j] << "}" ;
  }
  cout <<  std::endl; 
  cout << " qGoal: " <<  std::endl; 
  for(unsigned int j = 0; j < qGoal.size(); j++){
    cout << std::fixed << " (" << j << "){" << qGoal[j] << "}" ;
  }
  cout <<  std::endl; 
  cout << " jmax: " <<  std::endl; 
  for(unsigned int j = 0; j < jmax.size(); j++){
    cout << std::fixed << " (" << j << "){" << jmax[j] << "}" ;
  }
  cout <<  std::endl; 
  cout << " amax: " <<  std::endl; 
  for(unsigned int j = 0; j < amax.size(); j++){
    cout << std::fixed << " (" << j << "){" << amax[j] << "}" ;
  }
  cout <<  std::endl; 
  cout << " vmax: " <<  std::endl; 
  for(unsigned int j = 0; j < vmax.size(); j++){
    cout << std::fixed << " (" << j << "){" << vmax[j] << "}" ;
  }
  cout <<  std::endl; 
  for(unsigned int j = 0; j < traj.size(); j++){
    std::cout << "===========  Traj on axis " << j <<" ==========" <<  std::endl; 
    std::cout << "   Number of segments: " << traj[j].size() <<  std::endl; 
    std::cout << "   Duration:           " << duration <<  std::endl; 
    std::cout << "   Segments:" <<  std::endl; 
    for(unsigned int k=0; k<traj[j].size();k++) {
      std::cout << std::fixed << " (" << k << "){Ti= "<< traj[j][k].timeOnTraj <<" }{T=" << traj[j][k].time << " , J=" <<  traj[j][k].jerk << "}{IC.x="<< traj[j][k].IC.x << "}" <<  "}{IC.v="<< traj[j][k].IC.v << "}" << "}{IC.a="<< traj[j][k].IC.a << "}" << std::endl;
    }
  }

}

void SM_TRAJ::printQStart() 
{
  cout.precision(4);
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl; 
  cout<<  "          TRAJECTORY QSTART           " << std::endl;  
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl; 
  cout << " Number of joints: " << traj.size() <<  std::endl;
  cout << " trajId: " << trajId <<  std::endl; 
  cout << " qStart: " <<  std::endl; 
  for(unsigned int j = 0; j < qStart.size(); j++){
    cout << std::fixed << " (" << j << "){" << qStart[j] << "}" ;
  }
  cout <<  std::endl; 
}

void SM_TRAJ::printQGoal() 
{
  cout.precision(4);
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl; 
  cout<<  "          TRAJECTORY QGOAL            " << std::endl;  
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl; 
  cout << " Number of joints: " << traj.size() <<  std::endl;
  cout << " trajId: " << trajId <<  std::endl; 
  cout << " qGoal: " <<  std::endl; 
  for(unsigned int j = 0; j < qGoal.size(); j++){
    cout << std::fixed << " (" << j << "){" << qGoal[j] << "}" ;
  }
  cout <<  std::endl; 
}

int SM_TRAJ::append(SM_TRAJ_STR inTraj) 
{
  SM_SEG seg;
  if(traj.size() != (unsigned int)inTraj.nbAxis) {
    printf("ERROR SM_TRAJ:append() diffrent size in trajs\n");
    traj.clear();
    traj.resize(inTraj.nbAxis);
  }
  for(unsigned int i=0; i<traj.size(); i++) {
    for(int j=0; j<inTraj.traj[i].nbSeg; j++) {
      seg.timeOnTraj = 0.0;
      seg.lpId = inTraj.traj[i].seg[j].lpId;
      seg.timeOnTraj = inTraj.traj[i].seg[j].timeOnTraj;
      seg.time = inTraj.traj[i].seg[j].time;
      seg.jerk = inTraj.traj[i].seg[j].jerk;
      seg.IC.a = inTraj.traj[i].seg[j].ic_a;
      seg.IC.v = inTraj.traj[i].seg[j].ic_v;
      seg.IC.x = inTraj.traj[i].seg[j].ic_x;
      traj[i].push_back(seg);
    }
  }
  computeTimeOnTraj();
  return 0;
}

int SM_TRAJ::append(SM_TRAJ &smTraj) 
{
  SM_SEG seg;

  SM_TRAJ_STR inTraj;

  smTraj.convertToSM_TRAJ_STR(&inTraj);

  if(traj.size() != (unsigned int)inTraj.nbAxis) {
    printf("ERROR SM_TRAJ:append() diffrent size in trajs\n");
    traj.clear();
    traj.resize(inTraj.nbAxis);
  }
  for(unsigned int i=0; i<traj.size(); i++) {
    for(int j=0; j<inTraj.traj[i].nbSeg; j++) {
      seg.timeOnTraj = 0.0;
      seg.lpId = inTraj.traj[i].seg[j].lpId;
      seg.timeOnTraj = inTraj.traj[i].seg[j].timeOnTraj;
      seg.time = inTraj.traj[i].seg[j].time;
      seg.jerk = inTraj.traj[i].seg[j].jerk;
      seg.IC.a = inTraj.traj[i].seg[j].ic_a;
      seg.IC.v = inTraj.traj[i].seg[j].ic_v;
      seg.IC.x = inTraj.traj[i].seg[j].ic_x;
      traj[i].push_back(seg);
    }
  }
  computeTimeOnTraj();
  return 0;
}



int SM_TRAJ::save(char *name)
{
  FILE * fileptr = NULL;
  if ((fileptr = fopen(name,"w+"))==NULL) {
    printf("cannot open File %s", name);
    return 1;
  }

  /* Read File Variables */
  fprintf(fileptr, "%d\n", (int)trajId);
  //fprintf(fileptr, "%f\n", (double)timePreserved);
  fprintf(fileptr, "%d\n", (int)traj.size());

  for(unsigned int i=0; i<qStart.size(); i++) {
    fprintf(fileptr, "%f\t", qStart[i]);
  }
  fprintf(fileptr, "\n");
  for(unsigned int i=0; i<qGoal.size(); i++) {
    fprintf(fileptr, "%f\t", qGoal[i]);
  }
  fprintf(fileptr, "\n");
  for(unsigned int i=0; i<jmax.size(); i++) {
    fprintf(fileptr, "%f\t", jmax[i]);
  }
  fprintf(fileptr, "\n");
  for(unsigned int i=0; i<amax.size(); i++) {
    fprintf(fileptr, "%f\t", amax[i]);
  }
  fprintf(fileptr, "\n");
  for(unsigned int i=0; i<vmax.size(); i++) {
    fprintf(fileptr, "%f\t", vmax[i]);
  }
  fprintf(fileptr, "\n");
  for(unsigned int i=0; i<traj.size(); i++) {
    for(unsigned int j=0; j<traj[i].size(); j++) {
      fprintf(fileptr, "%d\t", traj[i][j].lpId);
      fprintf(fileptr, "%f\t", traj[i][j].timeOnTraj);
      fprintf(fileptr, "%f\t", traj[i][j].time);
      fprintf(fileptr, "%f\t", traj[i][j].IC.a);
      fprintf(fileptr, "%f\t", traj[i][j].IC.v);
      fprintf(fileptr, "%f\t", traj[i][j].IC.x);
      fprintf(fileptr, "%f\t", traj[i][j].jerk);
    }
    fprintf(fileptr, "\n");
  }
  if(fileptr) {
    fclose(fileptr);
  }
  return 0;
}


void SM_TRAJ::tokenize(const std::string& str,
		       std::vector<std::string>& tokens,
		       const std::string& delimiters = " "){
  // Skip delimiters at beginning.
  std::string::size_type lastPos = str.find_first_not_of(delimiters, 0);
  // Find first "non-delimiter".
  std::string::size_type pos = str.find_first_of(delimiters, lastPos);

  while (std::string::npos != pos || std::string::npos != lastPos) {
    // Found a token, add it to the vector.
    tokens.push_back(str.substr(lastPos, pos - lastPos));
    // Skip delimiters.  Note the "not_of"
    lastPos = str.find_first_not_of(delimiters, pos);
    // Find next "non-delimiter"
    pos = str.find_first_of(delimiters, lastPos);
  }
}

std::vector<double> SM_TRAJ::parseFrame(std::string& line){
  std::vector<double> frame;
  std::vector<std::string> stringVector;
  tokenize(line, stringVector, "\t");
  for (unsigned int i = 0; i < stringVector.size(); i++) {
    frame.push_back(atof(stringVector[i].c_str()));
  }
  return frame;
}

int SM_TRAJ::load(char *name) 
{
  ifstream file(name, ios::in);  // on ouvre en lecture
  std::vector<std::string> stringVector;
  int nbAxis = 0;
  //int nbSeg  = 0;
  string contenu;  // declaration d'une chane qui contiendra la ligne lue
  std::vector<double> doubleVector ;
  

  this->clear();

  if(file) {
    /* Read trajId */
    getline(file, contenu);
    doubleVector = parseFrame(contenu);
    trajId = doubleVector[0];

    ///* Read timePreserved */
    //getline(file, contenu);
    //doubleVector = parseFrame(contenu);
    //timePreserved = doubleVector[0];
    
    /* Read nbAxis */
    doubleVector.clear();
    getline(file, contenu);
    doubleVector = parseFrame(contenu);
    nbAxis = doubleVector[0];
    
    this->resize(nbAxis);

    /* Read qStart */
    doubleVector.clear();
    getline(file, contenu);
    doubleVector = parseFrame(contenu);
    for(unsigned int i=0; i <doubleVector.size(); i++) {
      qStart[i] = doubleVector[i];
    }
 
    /* Read qGoal */
    doubleVector.clear();
    getline(file, contenu);
    doubleVector = parseFrame(contenu);
    for(unsigned int i=0; i <doubleVector.size(); i++) {
      qGoal[i] = doubleVector[i];
    }

   /* Read jmax */
    doubleVector.clear();
    getline(file, contenu);
    doubleVector = parseFrame(contenu);
    for(unsigned int i=0; i <doubleVector.size(); i++) {
      jmax[i] = doubleVector[i];
    }

   /* Read amax */
    doubleVector.clear();
    getline(file, contenu);
    doubleVector = parseFrame(contenu);
    for(unsigned int i=0; i <doubleVector.size(); i++) {
      amax[i] = doubleVector[i];
    }

   /* Read vmax */
    doubleVector.clear();
    getline(file, contenu);
    doubleVector = parseFrame(contenu);
    for(unsigned int i=0; i <doubleVector.size(); i++) {
      vmax[i] = doubleVector[i];
    }
 
    for(int a=0; a<nbAxis; a++) {
      
      doubleVector.clear();
      getline(file, contenu);
      doubleVector = parseFrame(contenu);
      traj[a].resize((int)doubleVector.size()/7);
      for(unsigned int segIndex=0; segIndex<doubleVector.size()/7; segIndex++) {
	traj[a][segIndex].lpId       = doubleVector[7*segIndex +0];
	traj[a][segIndex].timeOnTraj = doubleVector[7*segIndex +1];
	traj[a][segIndex].time       = doubleVector[7*segIndex +2];
	traj[a][segIndex].IC.a       = doubleVector[7*segIndex +3];
	traj[a][segIndex].IC.v       = doubleVector[7*segIndex +4];
	traj[a][segIndex].IC.x       = doubleVector[7*segIndex +5];
	traj[a][segIndex].jerk       = doubleVector[7*segIndex +6];	
      }
    }
    file.close();
    this->computeTimeOnTraj();
    cout << "SM_TRAJ::load file " << name << " loaded with " << nbAxis << " axes" << endl;  

  } else {
    std::cout << "Cannot open the file " << name << " !" << endl;
  }
  return 0;
}

int SM_TRAJ::convertToSM_TRAJ_STR(SM_TRAJ_STR *smTraj)
{
  //  printf("ERROR nbAxis in traj : (%d) != SM_TRAJ_NB_AXIS\n", (int)traj.size());
  //  return 1;
  //}
  smTraj->trajId = this->trajId;
  smTraj->timePreserved = this->timePreserved;
  smTraj->nbAxis = (int)traj.size();

  if((int)this->traj.size() > SM_TRAJ_NB_AXIS) {
    printf("ERROR convertToSM_TRAJ_STR(): too many axes in traj: %d > %d\n", this->traj.size(), SM_TRAJ_NB_AXIS) ;
    return 1;
  }
  for(unsigned int i = 0; i< this->traj.size(); ++i) {
    if((int)this->traj[i].size() > SM_SEGMENT_TRACK_MAX_SIZE) {
      printf("ERROR convertToSM_TRAJ_STR(): too many segment in traj for axis %d : %d > %d\n",i, this->traj[i].size(), SM_SEGMENT_TRACK_MAX_SIZE); 
      return 1;
    }
  }
  for(int i=0; i<smTraj->nbAxis; i++) {
    smTraj->qStart[i] =  this->qStart[i];
  }
  for(int i=0; i<smTraj->nbAxis; i++) {
    smTraj->qGoal[i] =  this->qGoal[i];
  }

  for(int i=0; i<smTraj->nbAxis; i++) {
    smTraj->jmax[i] =  this->jmax[i];
  }

  for(int i=0; i<smTraj->nbAxis; i++) {
    smTraj->amax[i] =  this->amax[i];
  }

  for(int i=0; i<smTraj->nbAxis; i++) {
    smTraj->vmax[i] =  this->vmax[i];
  }

  for(int i=0; i<smTraj->nbAxis; i++) {
    smTraj->traj[i].nbSeg = (int)traj[i].size();
    for(int j=0; j<smTraj->traj[i].nbSeg; j++) {
      smTraj->traj[i].seg[j].lpId       =  traj[i][j].lpId;
      smTraj->traj[i].seg[j].timeOnTraj =  traj[i][j].timeOnTraj;
      smTraj->traj[i].seg[j].time       =  traj[i][j].time;
      smTraj->traj[i].seg[j].ic_a       =  traj[i][j].IC.a;
      smTraj->traj[i].seg[j].ic_v       =  traj[i][j].IC.v;
      smTraj->traj[i].seg[j].ic_x       =  traj[i][j].IC.x;
      smTraj->traj[i].seg[j].jerk       =  traj[i][j].jerk;
    }
  }
  return 0;
}


int SM_TRAJ::importFromSM_OUTPUT(int trajId, double sampling, std::vector<SM_OUTPUT> &trajIn)
{
  this->clear();

  if(trajIn.size() == 0) {
    printf("ERROR: importFromSM_OUTPUT empty traj\n");
    return 1;
  }
  if(trajIn[0].Time.size() == 0) {
    printf("ERROR: importFromSM_OUTPUT empty traj\n");
    return 1;
  }
  int nbPoint = (int)trajIn.size();
  int nbAxis = (int)trajIn[0].Time.size();

  //  printf("importFromSM_OUTPUT: There are %f axes and %f segments\n", (double)trajIn[0].Time.size(), (double)trajIn.size());

  this->resize(trajIn[0].Time.size());
  this->trajId = trajId;
  this->timePreserved = 0.0;
 
  for(int i=0; i<nbAxis; i++) {
    traj[i].resize(nbPoint);
    for(int j=0; j<nbPoint; j++) {
      traj[i][j].lpId       = 0;
      traj[i][j].timeOnTraj = trajIn[j].premier_point*sampling;
      traj[i][j].time       =  trajIn[j].Time[i];
      traj[i][j].IC.a	    =  trajIn[j].IC[i].a;
      traj[i][j].IC.v	    =  trajIn[j].IC[i].v;
      traj[i][j].IC.x	    =  trajIn[j].IC[i].x;
      traj[i][j].jerk	    =  trajIn[j].Jerk[i];
    }
  }

  if(trajIn[0].IC.size()) {
    for(int i=0; i<nbAxis; i++) {
      this->qStart[i] = trajIn[0].IC[i].x;
    }
    for(int i=0; i<nbAxis; i++) {
      this->qGoal[i] = 0.0;
    }
    for(int i=0; i<nbAxis; i++) {
      this->jmax[i] = 0.0;
    }
    for(int i=0; i<nbAxis; i++) {
      this->amax[i] = 0.0;
    }
    for(int i=0; i<nbAxis; i++) {
      this->vmax[i] = 0.0;
    }
  }

  /*set the initial condition in the first segment */
  for(int i=0; i<nbAxis; i++) {
    traj[i][0].IC.a	    =  trajIn[0].IC[i].a;
    traj[i][0].IC.v	    =  trajIn[0].IC[i].v;
    traj[i][0].IC.x	    =  trajIn[0].IC[i].x;
  }


  //this->print();
  // compute the initial conditions and other variable for all segments
  computeTimeOnTraj();

  //this->print();
  
  return 0;
}


int SM_TRAJ::convertToSM_OUTPUT(int trajId, double sampling, std::vector<SM_OUTPUT> &trajIn)
{
  computeTimeOnTraj();
  trajIn.clear();

  trajIn.resize(this->traj[0].size());
  trajIn[0].Time.resize(this->traj.size());

  int nbPoint = (int)trajIn.size();
  int nbAxis = (int)trajIn[0].Time.size();


  //  printf("importFromSM_OUTPUT: There are %f axes and %f segments\n", (double)trajIn[0].Time.size(), (double)trajIn.size());


  this->resize(trajIn[0].Time.size());
  this->trajId = trajId;
  this->timePreserved = 0.0;

  for(int i=0; i<nbAxis; i++) {
    traj[i].resize(nbPoint);
    for(int j=0; j<nbPoint; j++) {

      trajIn[j].premier_point= traj[i][j].timeOnTraj/sampling  ;
      trajIn[j].Time[i] = traj[i][j].time;
      trajIn[j].IC[i].a = traj[i][j].IC.a;
      trajIn[j].IC[i].v = traj[i][j].IC.v;
      trajIn[j].IC[i].x = traj[i][j].IC.x;
      trajIn[j].Jerk[i] = traj[i][j].jerk;
    }
  }

  if(trajIn[0].IC.size()) {
    for(int i=0; i<nbAxis; i++) {
      trajIn[0].IC[i].x = this->qStart[i];
    }
    //   for(int i=0; i<nbAxis; i++) {
    //     this->qGoal[i] = 0.0;
    //   }
  }

  /*set the initial condition in the first segment */
  for(int i=0; i<nbAxis; i++) {
    trajIn[0].IC[i].a = traj[i][0].IC.a ;
    trajIn[0].IC[i].v=  traj[i][0].IC.v ;
    trajIn[0].IC[i].x =  traj[i][0].IC.x ;
  }

  //this->print();
  // compute the initial conditions and other variable for all segments

  //this->print();

  return 0;
}





int SM_TRAJ::importFromSM_TRAJ_STR(const SM_TRAJ_STR *smTraj)
{
  //if(smTraj->nbAxis != SM_TRAJ_NB_AXIS) {
  //  printf("ERROR nbAxis in input traj : (%d) != SM_TRAJ_NB_AXIS\n", (int)smTraj->nbAxis);
  //  return 1;
  //}
  this->clear();
  this->resize(smTraj->nbAxis);
  this->trajId = smTraj->trajId;
  this->timePreserved = smTraj->timePreserved;
  for(int i=0; i<smTraj->nbAxis; i++) {
    this->qStart[i] = smTraj->qStart[i];
  }
  for(int i=0; i<smTraj->nbAxis; i++) {
    this->qGoal[i] = smTraj->qGoal[i];
  }
  for(int i=0; i<smTraj->nbAxis; i++) {
    this->jmax[i] = smTraj->jmax[i];
  }
  for(int i=0; i<smTraj->nbAxis; i++) {
    this->amax[i] = smTraj->amax[i];
  }
  for(int i=0; i<smTraj->nbAxis; i++) {
    this->vmax[i] = smTraj->vmax[i];
  }
  
  for(int i=0; i<smTraj->nbAxis; i++) {
    traj[i].resize(smTraj->traj[i].nbSeg);
    for(int j=0; j<smTraj->traj[i].nbSeg; j++) {
      traj[i][j].lpId       = smTraj->traj[i].seg[j].lpId      ;
      traj[i][j].timeOnTraj = smTraj->traj[i].seg[j].timeOnTraj;
      traj[i][j].time       = smTraj->traj[i].seg[j].time      ;
      traj[i][j].IC.a	    = smTraj->traj[i].seg[j].ic_a      ;
      traj[i][j].IC.v	    = smTraj->traj[i].seg[j].ic_v      ;
      traj[i][j].IC.x	    = smTraj->traj[i].seg[j].ic_x      ;
      traj[i][j].jerk	    = smTraj->traj[i].seg[j].jerk      ;
    }
  }

  this->computeTimeOnTraj();

  return 0;
}

//int SM_TRAJ::exportQFile(char *name)
//{
//  FILE * fileptr = NULL;
//  if ((fileptr = fopen(name,"w+"))==NULL) {
//    printf("cannot open File %s", name);
//    return 1;
//  }
//
//
//  for(int i=0; i<traj.size(); i++) {
//	fprintf(fileptr, "%f\t", (int)traj[i][j].lpId);
//	fprintf(fileptr, "%f\t", traj[i][j].timeOnTraj);
//	fprintf(fileptr, "%f\t", traj[i][j].time);
//	fprintf(fileptr, "%f\t", traj[i][j].IC.a);
//	fprintf(fileptr, "%f\t", traj[i][j].IC.v);
//	fprintf(fileptr, "%f\t", traj[i][j].IC.x);
//	fprintf(fileptr, "%f\t", traj[i][j].jerk);
//  }
//  fprintf(fileptr, "\n");
//
//    
//  if(fileptr) {
//    fclose(fileptr);
//  }
//
//
//
//}

int SM_TRAJ::extract(double t1, double t2, SM_TRAJ &trajIn)
{
  SM_TRAJ_STR trajStr;
  SM_TRAJ_STR trajStrOut;
    
  trajIn.computeTimeOnTraj();
  trajIn.convertToSM_TRAJ_STR(&trajStr);
  
  this->trajId = trajIn.trajId;
  this->timePreserved = trajIn.timePreserved;

  int id1 = trajIn.getSegmentIndex(t1);
  printf("t1 %f durtation %f\n", t1, trajIn.getDuration());
  int id2 = trajIn.getSegmentIndex(t2);
  printf("id1 %d id2 %d\n", id1, id2);
  double dt1 = t1 - trajStr.traj[0].seg[id1].timeOnTraj;
  double dt2 = t2 - trajStr.traj[0].seg[id2].timeOnTraj;
  
  std::vector< SM_COND > cond1;
  trajIn.getMotionCond(t1, cond1);
  
  std::vector< SM_COND > cond2;
  trajIn.getMotionCond(t2, cond2);
  
  int nbSeg = id2-id1 +1;
  
  trajStrOut.nbAxis = trajStr.nbAxis;
  for(int i=0; i< trajStr.nbAxis; i++) {
    trajStrOut.traj[i].nbSeg = nbSeg;
    for(int j=0; j< nbSeg; j++) {
      if(id1 < id2) {
	if(j ==0) {
	  trajStrOut.traj[i].seg[j].lpId      =0;
	  trajStrOut.traj[i].seg[j].timeOnTraj=0;
	  trajStrOut.traj[i].seg[j].time      = trajStr.traj[i].seg[id1].time  - dt1;
	  trajStrOut.traj[i].seg[j].ic_a      = cond1[i].a;
	  trajStrOut.traj[i].seg[j].ic_v      = cond1[i].v;
	  trajStrOut.traj[i].seg[j].ic_x      = cond1[i].x;
	  trajStrOut.traj[i].seg[j].jerk      = trajStr.traj[i].seg[id1].jerk;
	} else if(j ==nbSeg-1) {
      	  trajStrOut.traj[i].seg[j].lpId      =0;
	  trajStrOut.traj[i].seg[j].timeOnTraj=0;
	  trajStrOut.traj[i].seg[j].time      = dt2;
	  trajStrOut.traj[i].seg[j].ic_a      = cond2[i].a;
	  trajStrOut.traj[i].seg[j].ic_v      = cond2[i].v;
	  trajStrOut.traj[i].seg[j].ic_x      = cond2[i].x;
	  trajStrOut.traj[i].seg[j].jerk      = trajStr.traj[i].seg[id2].jerk;
	} else {
	  trajStrOut.traj[i].seg[j].lpId      =0;
	  trajStrOut.traj[i].seg[j].timeOnTraj=0;
	  trajStrOut.traj[i].seg[j].time      = trajStr.traj[i].seg[id1+j].time;
	  trajStrOut.traj[i].seg[j].ic_a      = trajStr.traj[i].seg[id1+j].ic_a;
	  trajStrOut.traj[i].seg[j].ic_v      = trajStr.traj[i].seg[id1+j].ic_v;
	  trajStrOut.traj[i].seg[j].ic_x      = trajStr.traj[i].seg[id1+j].ic_x;
	  trajStrOut.traj[i].seg[j].jerk      = trajStr.traj[i].seg[id1+j].jerk;
	}
      
      } else {
	trajStrOut.traj[i].seg[j].lpId      =0;
	trajStrOut.traj[i].seg[j].timeOnTraj=0;
	trajStrOut.traj[i].seg[j].time      = t2  - t1;
	trajStrOut.traj[i].seg[j].ic_a      = cond1[i].a;
	trajStrOut.traj[i].seg[j].ic_v      = cond1[i].v;
	trajStrOut.traj[i].seg[j].ic_x      = cond1[i].x;
	trajStrOut.traj[i].seg[j].jerk      = trajStr.traj[i].seg[id1].jerk;
      }

    }
  }
  this->clear();
  this->importFromSM_TRAJ_STR(&trajStrOut);
  this->computeTimeOnTraj();
  return 0;
}

int SM_TRAJ::getSegmentIndex(double t1)
{
  int id = 0;
  SM_TRAJ_STR trajStr;
  
  
  if(t1 > this->getDuration())
    {
      printf("ERROR getSegmentIndex: t1 > this.getDuration()\n");
      return 0 ;
    }
  
  this->convertToSM_TRAJ_STR(&trajStr);
  
  for(int i=0; i<  trajStr.traj[0].nbSeg; i++) {
    if((t1 >= trajStr.traj[0].seg[i].timeOnTraj) && (t1 < (trajStr.traj[0].seg[i].timeOnTraj + trajStr.traj[0].seg[i].time) ) ) {
      return i; 
    }
  } 
  return id; 
}

int SM_TRAJ::approximateSVGFile( double jmax,  double amax,  double vmax,  double SampTime, double ErrMax, char *fileName)
{

#ifdef QT_LIBRARY
  // bool flagExport = true;
  // QSoftMotionPlanner w;
  // int ExpTime = 10;
  // SM_TRAJ traj,
  // w.approximate(jmax, amax, vmax, SampTime,  ErrMax,  ExpTime, flagExport, fileName, traj);
  // this->clear();
#else
  printf("ERROR: softMotion-libs is not compiled with the QT_LIBRARY flag\n");
#endif
  return 0;
}


int SM_TRAJ::approximate(std::vector< std::vector<SM_COND> > &trajIn, double timeStep, double errorPosMax,double errorVelMax, int id, bool flag)
{

  this->clear();
  this->trajId = id;
  this->timePreserved = 0.0;

  if(trajIn.size() == 0) {
    printf("SmTraj: trajectory empty\n");
    return 1;
  }
  if(timeStep <= 0) {
    printf("SmTraj: error input timeStep\n");
    return 1;
  }

  this->resize(trajIn.size());

  int nbAxis = (int)trajIn.size();
  int nbSample = trajIn[0].size();

  double reste = ((nbSample-1)%3);
  int nbSampleAdjusted = 0;

  if(reste == 0) {
    nbSampleAdjusted = nbSample;
  } else {
    nbSampleAdjusted = nbSample + (3 - ((nbSample-1)%3)) ;
  }


  printf("INFO SM_TRAJ:: There are %d points, %d points adjusted and %d axes\n",nbSample, nbSampleAdjusted, nbAxis);
  double total_time = nbSampleAdjusted*timeStep;

  for(int i=0; i< nbAxis; i++) {
    this->qStart[i] = trajIn[i][0].x;
  }
  for(int i=0; i< nbAxis; i++) {
    this->qGoal[i] = trajIn[i][nbSample-1].x;
  }

  Sm_Curve curv; // initialize and fill the ideal curve
  curv.traj.resize(nbSampleAdjusted);

  for(unsigned int i=0; i<curv.traj.size(); i++) {
    curv.traj[i].Pos.resize(nbAxis);
    curv.traj[i].Vel.resize(nbAxis);
    curv.traj[i].Acc.resize(nbAxis);
    curv.traj[i].Jerk.resize(nbAxis);
  }
  for (unsigned int i = 0; i < curv.traj.size(); i++){
    curv.traj[i].t = i * timeStep;
    if (curv.traj[i].t >= total_time) {
      curv.traj[i].t =total_time;
    }
  }
  for (int i = 0; i < nbSample; i++){
    for(int j=0; j< nbAxis; j++) {
      curv.traj[i].Pos[j] = trajIn[j][i].x;
      curv.traj[i].Vel[j] = trajIn[j][i].v;
      curv.traj[i].Acc[j] = trajIn[j][i].a;
    }
  }

  for (int i = nbSample; i < nbSampleAdjusted; i++){
    for(int j=0; j< nbAxis; j++) {
      curv.traj[i].Pos[j] = trajIn[j][nbSample-1].x;
      curv.traj[i].Vel[j] = 0.0;
      curv.traj[i].Acc[j] = 0.0;
    }
  }

  // utilise la classe SM_Approx pour effectuer l'approximation
  Sm_Approx approx;
  int res = approx.approximate(curv, timeStep, errorPosMax, errorVelMax, *this, flag);

  return res;
}

int SM_TRAJ::computeMaxTimeScaleVectorTest(std::vector<double> & maxVel, double tic, SM_LIMITS timeLimits)
{
  SM_COND condTs;
  std::vector<double> tsVectTmp;
  std::vector<SM_COND> cond;
  std::vector<double> tsForward;
  std::vector<double> tsBackward;
  FILE* file = NULL;

  tsVectTmp.clear();
  tsVectTmp.resize(1000);

  for(int i=0; i<20; i++) {
    tsVectTmp[i] = 1.0;
  }
  for(int i=20; i<200; i++) {
    tsVectTmp[i] = 0.2;
  }
  for(int i=200; i<350; i++) {
    tsVectTmp[i] = 0.25;
  }
  for(int i=350; i<400; i++) {
    tsVectTmp[i] =1.0;
  }

  for(int i=400; i<700; i++) {
    tsVectTmp[i] = 0.25;
  }

  for(int i=700; i<1000; i++) {
    tsVectTmp[i] = 1.0;
  }


  file = fopen("toto.dat", "w");
  tsForward.resize(tsVectTmp.size());
  tsBackward.resize(tsVectTmp.size());

  /* compute the forward timeScale */
  condTs.a = 0.0;
  condTs.v = tsVectTmp[0];
  condTs.x = 0.0;
  tsForward[0] = tsVectTmp[0];

  for(uint i= 1; i< tsVectTmp.size(); i++) {
    if(tsVectTmp[i] > tsForward[i-1]) {
      sm_ComputeSmoothedStepVel(tsVectTmp[i] , tic, timeLimits, &condTs);
      tsForward[i] = condTs.v;
    } else {
      condTs.a = 0.0;
      condTs.v = tsVectTmp[i];
      condTs.x = 0.0;  
      tsForward[i] = tsVectTmp[i];
    }
  }

  /* compute the backward timeScale */
  condTs.a = 0.0;
  condTs.v = tsVectTmp[tsVectTmp.size()-1];
  condTs.x = 0.0;
  tsBackward[tsVectTmp.size()-1] = tsVectTmp[tsVectTmp.size()-1];

  for(int i= tsVectTmp.size()-2; i >=0; i--) {
    if(tsVectTmp[i] > tsBackward[i+1]) {
      sm_ComputeSmoothedStepVel(tsVectTmp[i] , tic, timeLimits, &condTs);
      tsBackward[i] = condTs.v;
    } else {
      condTs.a = 0.0;
      condTs.v = tsVectTmp[i];
      condTs.x = 0.0;  
      tsBackward[i] = tsVectTmp[i];
    }
  }

  tsVec.clear();
  tsVec.resize(tsVectTmp.size());

  for(uint i= 0; i< tsVectTmp.size(); i++) {
    tsVec[i] = MIN( tsForward[i],tsBackward[i]); 
  }

  for(uint i=0; i< tsVectTmp.size(); i++) {
    fprintf(file, "%f %f \n",tsVectTmp[i], tsVec[i] );
  }

  fclose(file);
  return 0;
}




int SM_TRAJ::computeMaxTimeScaleVector(std::vector<double> & maxVel, double tic, SM_LIMITS timeLimits)
{
  SM_COND condTs;

  if(maxVel.size() <= 0 || maxVel.size() > this->traj.size()) {
    printf("ERROR: SM_TRAJ::computeMaxTimeScaleVector wrong maxVel.size()\n");
    return 1;
  }
  std::vector<double> tsVectTmp;
  tsVectTmp.resize(duration/tic+1);
  
  for(uint i =0; i < tsVectTmp.size(); i++) {
    tsVectTmp[i] = 1.0;
  }

  double alpha, alphaMax;
  std::vector<SM_COND> cond;

  for(double time =0; time < duration; time = time + tic) {
    this->getMotionCond(time, cond);
    alphaMax = 1.0;
    for(uint i =0; i < maxVel.size(); i++) {
      if(cond[i].v > maxVel[i]) {
	alpha =  maxVel[i] / cond[i].v;
      } else {
	alpha = 1.0;
      }
      if(alpha < alphaMax) {
	alphaMax = alpha;
      }
    }
    tsVectTmp[time] = alphaMax;
  }

  std::vector<double> tsForward;
  std::vector<double> tsBackward;
  tsForward.resize(tsVectTmp.size());
  tsBackward.resize(tsVectTmp.size());

  /* compute the forward timeScale */
  condTs.a = 0.0;
  condTs.v = tsVectTmp[0];
  condTs.x = 0.0;

  tsForward[0] = tsVectTmp[0];

  for(uint i= 1; i< tsVectTmp.size(); i++) {
    if(tsVectTmp[i] > tsForward[i-1]) {
      sm_ComputeSmoothedStepVel(tsVectTmp[i] , tic, timeLimits, &condTs);
      tsForward[i] = condTs.v;
    } else {
      condTs.a = 0.0;
      condTs.v = tsVectTmp[i];
      condTs.x = 0.0;  
      tsForward[i] = tsVectTmp[i];
    }
  }

  /* compute the backward timeScale */
  condTs.a = 0.0;
  condTs.v = tsVectTmp[tsVectTmp.size()-1];
  condTs.x = 0.0;

  tsBackward[tsVectTmp.size()-1] = tsVectTmp[tsVectTmp.size()-1];

  for(int i= tsVectTmp.size()-2; i >=0; i--) {
    if(tsVectTmp[i] > tsBackward[i+1]) {
      sm_ComputeSmoothedStepVel(tsVectTmp[i] , tic, timeLimits, &condTs);
      tsBackward[i] = condTs.v;
    } else {
      condTs.a = 0.0;
      condTs.v = tsVectTmp[i];
      condTs.x = 0.0;  
      tsBackward[i] = tsVectTmp[i];
    }
  }

  tsVec.clear();
  tsVec.resize(tsVectTmp.size());

  for(uint i= 0; i< tsVectTmp.size(); i++) {
    tsVec[i] = MIN( tsForward[i],tsBackward[i]); 
  }

  return 0;
}


int SM_TRAJ::computeOneDimTraj(SM_COND IC, SM_COND FC, SM_LIMITS limits)
{ 
  SM_SEG seg;
  SM_STATUS resp;
  SM_TIMES T_Jerk;
  int dir = 0;


  SM_COND icl, fcl, fco;
  double GD;


  icl.a = IC.a;
  icl.v = IC.v;
  icl.x = 0.0;
  
  fcl.a = FC.a;
  fcl.v = FC.v;
  fcl.x = FC.x - IC.x;
  this->clear();
  this->resize(1);
 
  std::vector<double> I(3);
  std::vector<double> T(SM_NB_SEG);
  std::vector<double> J(SM_NB_SEG);

  SM_JERKS Jerks;
  SM_TIMES acc ;
  SM_TIMES vel ;
  SM_TIMES pos ;

  Jerks.sel = 1;
  Jerks.J1 = limits.maxJerk;
  Jerks.J2 = 0.0;
  Jerks.J3 = 0.0;
  Jerks.J4 = 0.0;

  /* compute the motion */
  resp = sm_ComputeSoftMotion(icl, fcl, limits, &T_Jerk, &dir);
  if (resp != SM_OK) {
    printf("ERROR sm_ComputeSoftMotion\n");
  }
  GD =  fcl.x * dir;
  if (sm_VerifyTimes(SM_DISTANCE_TOLERANCE_LINEAR , GD, Jerks, IC, dir, T_Jerk, &fco, &(acc), &(vel), &(pos), SM_ON)!=0) {
    printf("ERROR  Verify Times \n");
    return 1;
  }

  /* get the  position at the next tick */       
  I[0] = IC.a;
  I[1] = IC.v;
  I[2] = IC.x;
  
  T[0] = T_Jerk.Tjpa;
  T[1] = T_Jerk.Taca;
  T[2] = T_Jerk.Tjna;
  T[3] = T_Jerk.Tvc;
  T[4] = T_Jerk.Tjnb;
  T[5] = T_Jerk.Tacb;
  T[6] = T_Jerk.Tjpb;
    
  J[0] =   dir*limits.maxJerk;
  J[1] =   0.0;
  J[2] = - dir*limits.maxJerk;
  J[3] =   0.0;
  J[4] = - dir*limits.maxJerk;
  J[5] =   0.0;
  J[6] =   dir*limits.maxJerk;

  for (int j = 0; j < SM_NB_SEG; j++) {
    seg.IC = IC;
    seg.time = T[j];
    seg.jerk = J[j];
    traj[0].push_back(seg);     
  }
  qStart.push_back(IC.x);
  qGoal.push_back(FC.x);
  jmax.push_back(limits.maxJerk);
  amax.push_back(limits.maxAcc);
  vmax.push_back(limits.maxVel);

  computeTimeOnTraj();
  return 0;
}


//int SM_TRAJ::computePTPmotion(std::vector< double > start, std::vector< double > goal, std::vector< double > jmax,  std::vector< double > amax,  std::vector< double > vmax)

int SM_TRAJ::computeTraj(std::vector<SM_COND> IC, std::vector<SM_COND> FC, std::vector<SM_LIMITS> limits, SM_TRAJ_MODE mode) {
  //no imposed Duration
  std::vector<double> imposedDuration(IC.size());
  for(unsigned i=0; i != imposedDuration.size(); ++i) {
    imposedDuration.at(i) = 0.0;
  }
  if(mode == SM_3SEGMENT) {
    printf("SM:: Need imposed duration for 3 segments mode \n");
    return 1;
  } 
  return( computeTraj(IC, FC, limits,  mode, imposedDuration));
}

int SM_TRAJ::computeTraj(std::vector<SM_COND> IC, std::vector<SM_COND> FC, std::vector<SM_LIMITS> limits, SM_TRAJ_MODE mode, std::vector<double> imposedDuration) {

  
unsigned int nb_dofs = IC.size();
  
  switch(mode) {
  case SM_INDEPENDANT: {
    
      this->clear();
      
      std::vector<SM_MOTION_AXIS> motion_arr;

    
      if(FC.size() != nb_dofs) {
	printf("ERROR SM_TRAJ::computeTraj FC.size() != nb_dofs\n");
	return 1;
      }
      if(limits.size() != nb_dofs) {
	printf("ERROR SM_TRAJ::computeTraj limits.size() != nb_dofs\n");
	return 1;
      }
      
      /* resize the array to store the motions */
      motion_arr.resize(IC.size());
    
      /* Fill the motion_arr */
      for (unsigned i=0; i < nb_dofs; i++) {
	motion_arr[i].limits = limits[i];
	motion_arr[i].jerk.J1 = limits[i].maxJerk;
	motion_arr[i].jerk.sel = 1;
	motion_arr[i].ic = IC[i];
	motion_arr[i].fc = FC[i];
	motion_arr[i].ic_rel.a =  IC[i].a;
	motion_arr[i].ic_rel.v =  IC[i].v;
	motion_arr[i].ic_rel.x =  0.0;
	motion_arr[i].fc_rel.a =  FC[i].a;
	motion_arr[i].fc_rel.v =  FC[i].v;
	motion_arr[i].fc_rel.x =  FC[i].x - IC[i].x;
	motion_arr[i].motion_duration = imposedDuration.at(i);
      }
      if(computeUnsynchronizedMotion(motion_arr) != 0) {
	printf("ERROR cannot compute unsynchronized motion\n");
	return 1;
      }
      checkTrajBounds(0.1, IC, FC);
      if(fillFromMotionArr(motion_arr) != 0) {
	printf("ERROR SM_TRAJ::fillFromMotionArr\n");
	return 1;
      }

      break;
  }
  case SM_SYNCHRONIZED: {
    
      this->clear();
      
      std::vector<SM_MOTION_AXIS> motion_arr;
      //int nb_dofs = IC.size();
    
      if(FC.size() != nb_dofs) {
	printf("ERROR SM_TRAJ::computeTraj FC.size() != nb_dofs\n");
	return 1;
      }
      if(limits.size() != nb_dofs) {
	printf("ERROR SM_TRAJ::computeTraj limits.size() != nb_dofs\n");
	return 1;
      }
      
      /* resize the array to store the motions */
      motion_arr.resize(IC.size());
    
      /* Fill the motion_arr */
      for (unsigned i=0; i < nb_dofs; i++) {
	motion_arr[i].limits = limits[i];
	motion_arr[i].jerk.J1 = limits[i].maxJerk;
	motion_arr[i].jerk.sel = 1;
	motion_arr[i].ic = IC[i];
	motion_arr[i].fc = FC[i];
	motion_arr[i].ic_rel.a =  IC[i].a;
	motion_arr[i].ic_rel.v =  IC[i].v;
	motion_arr[i].ic_rel.x =  0.0;
	motion_arr[i].fc_rel.a =  FC[i].a;
	motion_arr[i].fc_rel.v =  FC[i].v;
	motion_arr[i].fc_rel.x =  FC[i].x - IC[i].x;
	motion_arr[i].motion_duration = imposedDuration.at(i);
      }
      if(computeSynchronizedMotion(motion_arr) != 0) {
	printf("ERROR cannot compute unsynchronized motion\n");
	return 1;
      }
        checkTrajBounds(0.1, IC, FC);
	if(fillFromMotionArr(motion_arr) != 0) {
	  printf("ERROR SM_TRAJ::fillFromMotionArr\n");
	  return 1;
	}
	break;
      }
    case SM_3SEGMENT: {
      
      std::vector<SM_COND_DIM> IC_3seg(1);
      std::vector<SM_COND_DIM> FC_3seg(1);
      std::vector<double> Timp(1);
      
      
      std::vector<SM_OUTPUT> motion;
      //int nb_dofs = IC.size();
      
      if(FC.size() != nb_dofs) {
	printf("ERROR SM_TRAJ::computeTraj FC.size() != nb_dofs\n");
	return 1;
      }
      if(limits.size() != nb_dofs) {
	printf("ERROR SM_TRAJ::computeTraj limits.size() != nb_dofs\n");
	return 1;
      }
      
      /* resize the array to store the motions */
      IC_3seg[0].Axis.resize(nb_dofs);
      FC_3seg[0].Axis.resize(nb_dofs);
      IC_3seg[0].Axis = IC;
      FC_3seg[0].Axis = FC;
      
      Timp.resize(nb_dofs);
      if( imposedDuration.size() != nb_dofs) {
	std::cout << "Error in SM: imposedDuration for traj different from DOF" << std::endl;
	return 1;
      }
      
      for(unsigned int i = 0; i != nb_dofs; ++i) {
	Timp.at(i) = imposedDuration.at(i);
      }
      
      motion.resize(3); // 3 segment, only one traj
	// motion.size() is size of segments, so for N traj, gives 3*N segments in total
	// motion.Jerk.size() is size of DOFs
      
      
	//
	// problem: in other usages of vector<SM_OUTPUT>,
	// like: importFromSM_OUTPUT()
	// size() gives nbPoints 
	// 
	//
	//
      
      
      
      
      SM_STATUS status;
      
      status = sm_SolveWithoutOpt(IC_3seg, FC_3seg, Timp, motion);
      
      if( status == SM_ERROR) {
	std::cout << "Error in SM when computing 3 segments trajectory" << std::endl;
	return 1;
      }

      // trajId?
      int trajId = 0;
      // sampling?
      double sampling = 0.001; //1ms, or to be tic* 1/10
      // premier_point
      
      if(importFromSM_OUTPUT(trajId, sampling, motion)!=0) {
	  
	
	//    if(ThreeSegSynchronizedMotion()!= 0) {
	// printf("ERROR SM_TRAJ::ThreeSegSynchronizedMotion\n");
	//  return 1;
	// }
	std::cout << "Error in SM: import from SM_OUTPUT" << std::endl;

      } else {
	checkTrajBounds(0.1, IC, FC);
      }
      
      
      std::vector<double> tmpMaxJerk(nb_dofs);
      std::vector<double> tmpMaxAcc(nb_dofs);
      std::vector<double> tmpMaxVel(nb_dofs);
      
      std::vector<SM_COND> cond_IC(nb_dofs);
      std::vector<SM_COND> cond_I(nb_dofs);
      std::vector<SM_COND> cond_II(nb_dofs); // two intermedia points to verify
      std::vector<SM_COND> cond_FC(nb_dofs);
      
      for (unsigned int j=0; j != nb_dofs; j++) {
	cond_IC[j] = IC[j];
	cond_I[j].a = motion[0].Jerk[j] * Timp[j] / 3  + cond_IC[j].a;
	cond_I[j].v = motion[0].Jerk[j] * pow(Timp[j]/3, 2.0) + cond_IC[j].a * Timp[j]/3 + cond_IC[j].v;
	cond_I[j].x = motion[0].Jerk[j] * pow(Timp[j]/3, 6.0) + cond_IC[j].a * pow(Timp[j]/3, 2.0) /2.0 + cond_IC[j].v * Timp[j]/3 + cond_IC[j].x;

	cond_II[j].a = motion[1].Jerk[j] * Timp[j] / 3  + cond_I[j].a;
	cond_II[j].v = motion[1].Jerk[j] * pow(Timp[j]/3, 2.0) + cond_I[j].a * Timp[j]/3 + cond_I[j].v;
	cond_II[j].x = motion[1].Jerk[j] * pow(Timp[j]/3, 6.0) + cond_I[j].a * pow(Timp[j]/3, 2.0) /2.0 + cond_I[j].v * Timp[j]/3 + cond_IC[j].x;

	cond_FC[j].a = motion[2].Jerk[j] * Timp[j] / 3  + cond_II[j].a;
	cond_FC[j].v = motion[2].Jerk[j] * pow(Timp[j]/3, 2.0) + cond_II[j].a * Timp[j]/3 + cond_II[j].v;
	cond_FC[j].x = motion[2].Jerk[j] * pow(Timp[j]/3, 6.0) + cond_II[j].a * pow(Timp[j]/3, 2.0) /2.0 + cond_II[j].v * Timp[j]/3 + cond_IC[j].x;

      }
	//calculate max jerk, acc, and vel
	for (unsigned int i=0; i != 3; i++) {
	  for (unsigned int j=0; j != nb_dofs; j++) {
	    if(ABS(motion[i].Jerk[j]) > tmpMaxJerk[j])
	      tmpMaxJerk[j] = ABS(motion[i].Jerk[j]);
	    //find max values in cond_I, cond_II, cond_FC, of acc and vel
	    if(ABS(cond_IC[j].a) > tmpMaxAcc[j])
	      tmpMaxAcc[j] =  ABS(cond_IC[j].a);
	    if(ABS(cond_I[j].a) > tmpMaxAcc[j])
	      tmpMaxAcc[j] =  ABS(cond_I[j].a);
	    if(ABS(cond_II[j].a) > tmpMaxAcc[j])
	      tmpMaxAcc[j] =  ABS(cond_II[j].a);
	    if(ABS(cond_FC[j].a) > tmpMaxAcc[j])
	      tmpMaxAcc[j] =  ABS(cond_FC[j].a);
	    
	    if(ABS(cond_IC[j].v) > tmpMaxVel[j])
	      tmpMaxVel[j] =  ABS(cond_IC[j].v);
	    if(ABS(cond_I[j].v) > tmpMaxVel[j])
	      tmpMaxVel[j] =  ABS(cond_I[j].v);
	    if(ABS(cond_II[j].v) > tmpMaxVel[j])
	      tmpMaxVel[j] =  ABS(cond_II[j].v);
	    if(ABS(cond_FC[j].v) > tmpMaxVel[j])
	      tmpMaxVel[j] =  ABS(cond_FC[j].v);
	    
              }
	}
	
	
	
	
	break;
    }
    
    default:
      printf("ERROR  SM_TRAJ::computeTraj wrong mode\n");
      return 1;
    }
      

      switch(mode) {
      case SM_INDEPENDANT:
      for (unsigned int i=0; i < nb_dofs; i++) {
        this->jmax[i] = limits[i].maxJerk;
        this->amax[i] = limits[i].maxAcc;
        this->vmax[i] = limits[i].maxVel;
      }
      break;
      case SM_3SEGMENT:
	

	// here to calculate jmax, amax and vmax
	/*
	  std::vector<SM_COND> cond_IC(nb_dofs);
	  std::vector<SM_COND> cond_I(nb_dofs);
	  std::vector<SM_COND> cond_II(nb_dofs);
	  std::vector<SM_COND> cond_FC(nb_dofs);
	  
      cond_IC = IC;
      cond_FC = FC;
      
      for(unsigned int i=0; i != nb_dofs; ++i){
      condI[i].a =
      }

      
      //IC.a =  jerk * dt  + ICl.a;
      // IC.v =  jerk * pow(dt,2.0) / 2.0 + ICl.a * dt   + ICl.v;
      // IC.x =  jerk * pow(dt,3.0) / 6.0 + ICl.a * pow(dt,2.0) / 2.0 + ICl.v * dt  + ICl.x;
      */
      break;
      case SM_SYNCHRONIZED:
	for (unsigned int i=0; i < nb_dofs; i++) {
	  this->jmax[i] = limits[i].maxJerk;
	  this->amax[i] = limits[i].maxAcc;
	  this->vmax[i] = limits[i].maxVel;
	}
      break;
      default:
	printf("ERROR  SM_TRAJ::computeTraj wrong mode\n");
	return 1;
      }
      
  
      return 0;
  }
  
  int SM_TRAJ::computeUnsynchronizedMotion(std::vector<SM_MOTION_AXIS> &motion_arr)
  {
    int nb_dofs = motion_arr.size();
    SM_COND FCm;
  
  if(nb_dofs <= 0) {
    printf("ERROR  SM_TRAJ::computeUnsynchronizedMotion nb_dofs <= 0\n");
    return 1;
  }
  
  for (int i=0; i < nb_dofs; i++) {    
    /* Compute the monodimensional trajectory */
    if (sm_ComputeSoftMotion( motion_arr[i].ic_rel, motion_arr[i].fc_rel, 
			      motion_arr[i].limits, &(motion_arr[i].times), 
			      &(motion_arr[i].dir))!=0) {
      printf("ERROR Jerk Profile on dim %d\n",i);
      return 1;
    }

    /* Compute trajectory lenght */
    sm_sum_motionTimes(&(motion_arr[i].times), &(motion_arr[i].motion_duration));

    /* Get initial conditions for each vectors Acc Vel and Pos */
    double GD =  motion_arr[i].fc_rel.x * motion_arr[i].dir;
    if (sm_VerifyTimes( SM_DISTANCE_TOLERANCE, GD,  motion_arr[i].jerk, 
			motion_arr[i].ic, motion_arr[i].dir, 
			motion_arr[i].times, &FCm, &(motion_arr[i].acc), 
			&(motion_arr[i].vel), &(motion_arr[i].pos), SM_ON)!=0) {
      printf(" Verify Times on dim %d\n",i);
      return 1;
    } 
  }
  return 0;
}






int SM_TRAJ::computeSynchronizedMotion(std::vector<SM_MOTION_AXIS> &motion_arr)
{
    //to implement
    return 0;
}

int SM_TRAJ::computeTraj(std::vector< std::vector<double> > via_points, std::vector<SM_LIMITS> limits, SM_TRAJ_TYPE type)
{
  std::vector<SM_TRAJ> smTrajs;
  SM_TRAJ smTraj;
  

  if(via_points.size() < 2) {
    cout << "ERROR computeTraj via_points.size() < 2 " << endl;
    return 1;
  }

  smTrajs.clear();
  for(unsigned int i=0; i< via_points.size()-1; i++) {
    smTraj.clear();
    //    if(smTraj.computeTraj(conds[i], conds[i+1], limits, SM_PTP) != 0) {
    // cout << "ERROR computeTraj cannot compute traj betewwen configs "<< i << " and " << i+1 << endl; 
      //  return 1;
      // }
    smTrajs.push_back(smTraj);
  }


  this->clear();
 // for (unsigned int i=0; i < nb_dofs; i++) {
 //   this->jmax[i] = limits[i].maxJerk;
 //   this->amax[i] = limits[i].maxAcc;
 //   this->vmax[i] = limits[i].maxVel;
 // }

  switch(type) { 
  case SM_STOP_AT_VIA_POINT:
    // concat all the ptp trajectories
    for(unsigned int i=0; i<smTrajs.size(); i++) {
      this->append(smTrajs[i]);
    }
    break;
  case SM_SMOOTH_AT_VIA_POINT:

    break;
  default:
    cout << "ERROR computeTraj wrong SM_TRAJ_TYPE" << endl;
    return 1;
  }




  return 0;
}



//
//   compute 3 segments motion
//
//   general synchronized motion: to be implemented
//


// implement: to use sm_SolveWithoutOpt() to compute a 3 segments trajectory


//SM_STATUS sm_SolveWithoutOpt(std::vector<SM_COND_DIM3> &IC, std::vector<SM_COND_DIM3> &FC, std::vector<double> &Timp, std::vector<SM_OUTPUT> &motion){
  /* This funciton compute the motion using 3 segment method without any optimization
     -- IC[]   : Initial condition array ( already discretized)
     -- FC[]   : Final condition array ( already discretized)
     -- Timp[] : Imposed time for each interval
     -- nbIntervals : number of discretized intervals
     -- motion[] : array of output command to the robot (composed of jerk and time duration for each axis)
  */
//

// which is used in class:
// used by class Sm_Approx::()
// adapt it to SM_Traj


// int SM_TRAJ::ThreeSegSynchronizedMotion(std::vector<SM_OUTPUT> &motion)
// {
//   SM_STATUS status;
  
//   status = sm_SolveWithoutOpt(IC, FC, Timp, motion);
  
//   return 0;
  
//  }

  //  int axis_motion_max = 0;
  //  double GD = 0.0;
  //  int i = 0;
  //  double time_motion_max = 0.0;
  //  int adjustTimeError = 0;
  //  SM_MOTION_MONO *motion3seg = NULL;
  //
  //
  //  SM_COND FCm;
  //
  //
  //
  //  motion3seg = new SM_MOTION_MONO[nb_dofs];
  //
  //
  //  for(i=0; i< nb_dofs; i++) { 
  //    
  //    if (motion_arr[i].motion_duration > time_motion_max) {
  //      time_motion_max = motion_arr[i].motion_duration;
  //      axis_motion_max = i;
  //    }
  //  }
  //
  //  /* Adjust Motion Times */
  //  adjustTimeError = 0;
  //
  //  for (i=0; i < nb_dofs; i++) {
  //    //if (i != axis_motion_max) {
  //
  //      ICm.a = IC[i].a;
  //      ICm.v = IC[i].v;
  //      ICm.x =  0.0;
  //      FCm.a = FC[i].a;
  //      FCm.v = FC[i].v;
  //      FCm.x = (FC[i].x - IC[i].x);
  //
  //      if(sm_adjustMotionWith3seg(motion_arr[i].ic_rel, motion_arr[i].fc_rel, , time_motion_max, &motion3seg[i])!= 0) {
  //	printf("sm_AdjustTime ERROR 3seg at axis %d\n",i);
  //	adjustTimeError ++;
  //      }
  //    // }
  //  }
  //

    //1

    // Sm_Approx approx;
    // int res = approx.approximate(curv, timeStep, errorPosMax, errorVelMax, *this, flag);

    //2 or
    //implement as in approx: to use sm_SolveWithoutOpt


  //
  //  if (adjustTimeError > 0) {
  //    printf("ERROR can't adjust time motion \n");
  //    return 1;
  //  }
  //
  //  for (i=0; i < nb_dofs; i++) {
  //    ICm.a =  IC[i].a;
  //    ICm.v =  IC[i].v;
  //    ICm.x =  IC[i].x;
  //    FCm.a =  FC[i].a;
  //    FCm.v =  FC[i].v;
  //    FCm.x = (FC[i].x - IC[i].x);
  //    /* Verify Times */
  //    if (sm_VerifyTimes_Dir_ab(SM_DISTANCE_TOLERANCE, FCm.x, t_jerk[i], ICm,
  //			      softMotion_data->specific->motion[i].Dir_a, softMotion_data->specific->motion[i].Dir_b,
  //			      softMotion_data->specific->motion[i].Times, &FC, &(softMotion_data->specific->motion[i].Acc),
  //			      &(softMotion_data->specific->motion[i].Vel), &(softMotion_data->specific->motion[i].Pos)) != 0) {
  //      printf("lm_compute_softMotion_for_r6Arm ERROR Verify Times on axis [%d] \n",i);
  //      return FALSE;
  //    }
  //    // 				softMotion_data->freeflyer->motion.motionIsAdjusted[i] = 1;
  //  }
  //





int SM_TRAJ::fillFromMotionArr(std::vector<SM_MOTION_AXIS> &motion_arr)
{
  int nb_dofs = motion_arr.size();
  SM_SEG seg;
  if(nb_dofs <= 0) {
    printf("ERROR  SM_TRAJ::fillFromMotionArr nb_dofs <= 0\n");
    return 1;
  }

  this->clear();
  this->resize(nb_dofs);
  
  for (int i=0; i < nb_dofs; i++) {
    this->qStart[i] = motion_arr[i].ic.x;
    this->qGoal[i] = motion_arr[i].fc.x;
    
    this->traj[i].clear();

    /* Fill first segment */
    seg.lpId = 0;
    seg.timeOnTraj = 0;
    seg.time =  motion_arr[i].times.Tjpa;
    seg.IC.a =  motion_arr[i].ic.a ;
    seg.IC.v =  motion_arr[i].ic.v;
    seg.IC.x =  motion_arr[i].ic.x;
    seg.jerk =  motion_arr[i].jerk.J1 *   motion_arr[i].dir;
    this->traj[i].push_back(seg);

    /* Fill second segment */
    seg.lpId = 0;
    seg.timeOnTraj = 0.0;
    seg.time =  motion_arr[i].times.Taca;
    seg.IC.a =  motion_arr[i].acc.Tjpa ;
    seg.IC.v =  motion_arr[i].vel.Tjpa;
    seg.IC.x =  motion_arr[i].pos.Tjpa;
    seg.jerk =  0.0;
    this->traj[i].push_back(seg);

    /* Fill third segment */
    seg.lpId = 0;
    seg.timeOnTraj = 0.0;
    seg.time =  motion_arr[i].times.Tjna;
    seg.IC.a =  motion_arr[i].acc.Taca ;
    seg.IC.v =  motion_arr[i].vel.Taca;
    seg.IC.x =  motion_arr[i].pos.Taca;
    seg.jerk =  -motion_arr[i].jerk.J1 *  motion_arr[i].dir;
    this->traj[i].push_back(seg);

    /* Fill fourth segment */
    seg.lpId = 0;
    seg.timeOnTraj = 0.0;
    seg.time =  motion_arr[i].times.Tvc;
    seg.IC.a =  motion_arr[i].acc.Tjna ;
    seg.IC.v =  motion_arr[i].vel.Tjna;
    seg.IC.x =  motion_arr[i].pos.Tjna;
    seg.jerk =  0.0;
    this->traj[i].push_back(seg);

    /* Fill fifth segment */
    seg.lpId = 0;
    seg.timeOnTraj = 0.0;
    seg.time =  motion_arr[i].times.Tjnb;
    seg.IC.a =  motion_arr[i].acc.Tvc ;
    seg.IC.v =  motion_arr[i].vel.Tvc;
    seg.IC.x =  motion_arr[i].pos.Tvc;
    seg.jerk =  -motion_arr[i].jerk.J1 *   motion_arr[i].dir;
    this->traj[i].push_back(seg);

    /* Fill sixth segment */
    seg.lpId = 0;
    seg.timeOnTraj = 0.0;
    seg.time =  motion_arr[i].times.Tacb;
    seg.IC.a =  motion_arr[i].acc.Tjnb ;
    seg.IC.v =  motion_arr[i].vel.Tjnb;
    seg.IC.x =  motion_arr[i].pos.Tjnb;
    seg.jerk =  0.0;
    this->traj[i].push_back(seg);

    /* Fill seventh segment */
    seg.lpId = 0;
    seg.timeOnTraj = 0.0;
    seg.time =  motion_arr[i].times.Tjpb;
    seg.IC.a =  motion_arr[i].acc.Tacb ;
    seg.IC.v =  motion_arr[i].vel.Tacb;
    seg.IC.x =  motion_arr[i].pos.Tacb;
    seg.jerk =  motion_arr[i].jerk.J1 *  motion_arr[i].dir;
    this->traj[i].push_back(seg);
  }
  this->computeTimeOnTraj();
  this->updateIC();
  return 0;
}

/*
Plot evolution of the position for each axis
 */
int SM_TRAJ::plot()
{
  ofstream myfile;
  myfile.open ("smTemp.dat");
  std::vector<SM_COND> cond;
  for(double tps=0.0; tps < this->duration; tps += 0.01) {
    this->getMotionCond(tps, cond);
    myfile << tps ;
    for(unsigned int i=0; i<cond.size(); i++) {
      myfile << " " << cond[i].x ;
    }
    myfile << endl;
  }
  myfile.close();
  
  try
    {
      Gnuplot g1("lines");
      g1.reset_plot();
      g1.cmd((char*)"set term wxt");
      //    g1.set_xrange(0, width);
      //    g1.set_yrange(0, -1*height);
      g1.cmd("set size ratio -1");
      g1.set_xautoscale();
      g1.set_yautoscale();
      g1.set_grid();
      for(unsigned int i=0; i<cond.size(); i++) {
	g1.plotfile_xy("smTemp.dat", 1, i+2);
      }
      wait_for_key();
    }
  catch (GnuplotException ge)
    {
      cout << ge.what() << endl;
    }
  return 0;

}

int SM_TRAJ::plot(int i)
{
  ofstream myfile;
  myfile.open ("smTemp.dat");
  std::vector<SM_COND> cond;
  for(double tps=0.0; tps < this->duration; tps += 0.01) {
    this->getMotionCond(tps, cond);
    myfile << tps << " " << cond[i].a << " " << cond[i].v << " " << cond[i].x << endl;
  }
  myfile.close();
  
  try
    {
      Gnuplot g1("lines");
      g1.reset_plot();
      g1.cmd((char*)"set term wxt");
      //    g1.set_xrange(0, width);
      //    g1.set_yrange(0, -1*height);
      g1.cmd("set size ratio -1");
      g1.set_xautoscale();
      g1.set_yautoscale();
      g1.set_grid();
      g1.plotfile_xy("smTemp.dat", 1, 2);
      g1.plotfile_xy("smTemp.dat", 1, 3);
      g1.plotfile_xy("smTemp.dat", 1, 4);
      wait_for_key();
    }
  catch (GnuplotException ge)
    {
      cout << ge.what() << endl;
    }
  return 0;
}


int SM_TRAJ::mergetwotrajectories( SM_TRAJ &Trajinitial, SM_TRAJ &Trajfinal)
{
   // SM_COND IC; // Conditions initiales
   // SM_COND FC; // Conditions finales
   // SM_LIMITS limits; // Valeurs max de Vel Acc et Jerk

    // On donne une ID a la nouvelle trajectoire Id de trajfinal+1 ou +2 si deja prise par Trajinitial

    this->trajId =0;

    // Le nouveau point qStart c'est le qStart de la premiere trajectoire et le nouveau qGoal c'est le qGoal de la seconde traj
    this->qStart = Trajfinal.qStart;
    this->qGoal = Trajinitial.qGoal;

    /* On determine jmax amax et vmax de la trajectoire resultante en prenant
     la plus grande valeur des max des deux trajectoires que l'on veut fusionner*/

        if(Trajfinal.jmax<Trajinitial.jmax)
            {
            this->jmax=Trajinitial.jmax;
            }
        else if(Trajfinal.jmax>=Trajinitial.jmax) {
            this->jmax=Trajfinal.jmax;
        }

        if(Trajfinal.amax<Trajinitial.amax)
            {
            this->amax=Trajinitial.amax;
            }
        else if(Trajfinal.amax>=Trajinitial.amax) {
            this->amax=Trajfinal.amax;
        }

        if(Trajfinal.vmax<Trajinitial.vmax)
            {
            this->vmax=Trajinitial.vmax;
            }
        else if(Trajfinal.vmax>=Trajinitial.vmax) {
            this->vmax=Trajfinal.vmax;
        }


     // La on met une traj a la suite de l'autre.

        SM_SEG seg;

        SM_TRAJ_STR inTraj;
        SM_TRAJ_STR inTraj2;

        Trajfinal.convertToSM_TRAJ_STR(&inTraj);

        if(traj.size() != (unsigned int)inTraj.nbAxis) {
          traj.clear();
          traj.resize(inTraj.nbAxis);
        }
        for(unsigned int i=0; i<traj.size(); i++) {
          for(int j=0; j<inTraj.traj[i].nbSeg; j++) {
            seg.timeOnTraj = 0.0;
            seg.lpId = inTraj.traj[i].seg[j].lpId;
            seg.timeOnTraj = inTraj.traj[i].seg[j].timeOnTraj;
            seg.time = inTraj.traj[i].seg[j].time;
            seg.jerk = inTraj.traj[i].seg[j].jerk;
            seg.IC.a = inTraj.traj[i].seg[j].ic_a;
            seg.IC.v = inTraj.traj[i].seg[j].ic_v;
            seg.IC.x = inTraj.traj[i].seg[j].ic_x;
            traj[i].push_back(seg);
          }
        }


        Trajinitial.convertToSM_TRAJ_STR(&inTraj2);

        for(unsigned int i=0; i<traj.size(); i++) {
          for(int j=0; j<inTraj2.traj[i].nbSeg; j++) {
            seg.timeOnTraj = 0.0;
            seg.lpId = inTraj2.traj[i].seg[j].lpId;
            seg.timeOnTraj = inTraj2.traj[i].seg[j].timeOnTraj;
            seg.time = inTraj2.traj[i].seg[j].time;
            seg.jerk = inTraj2.traj[i].seg[j].jerk;
            seg.IC.a = inTraj2.traj[i].seg[j].ic_a;
            seg.IC.v = inTraj2.traj[i].seg[j].ic_v;
            seg.IC.x = inTraj2.traj[i].seg[j].ic_x;
            traj[i].push_back(seg);
          }
        }
 // On calcule la duree de la trajectoire
        this->computeTimeOnTraj();

    this->timePreserved = 0.0;

     // OK ON A LA TRAJECTOIRE EN ENTIER


    // Il faut que je determine les valeur de M1 et M2 les deux points de comutations Voir si mon idee est bonne sur le calcule du temps de transitions.
    // On obtiendra alors les conditions initiales et les conditions finales

    // Et il faut calculer la courbe.
    return 0;
}
