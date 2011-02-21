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
  Auteur               : Xavier BROQUERE49

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
#endif

#include "Sm_Traj.h"
#include "Sm_Curve.h"
#include "Sm_Approx.h"
//#include "QSoftMotionPlanner.h"


using namespace std;

void SM_TRAJ::sm_traj()
{
  this->clear();
  return;
}

void SM_TRAJ::sm_traj(const SM_TRAJ &traj)
{
  this->trajId = traj.trajId;
  this->duration = traj.duration;
  this->qStart = traj.qStart;
  this->qGoal = traj.qGoal;
  this->traj = traj.traj;
  return;
}

void SM_TRAJ::clear() 
{
  qStart.clear();
  qGoal.clear();
  traj.clear();
  duration = 0.0;
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
  traj.resize(size);
  return;
}

int SM_TRAJ::getMotionCond(double time,std::vector<SM_COND> & cond)
{

  SM_COND IC;
  SM_STATUS resp;

  //   std::vector<double> t(1);
  //   std::vector<double> a(1);
  //   std::vector<double> v(1);
  //   std::vector<double> x(1);

  cond.clear();
  double dt = 0;
  SM_COND ICl;
  int idSeg = 0;
  double jerk =0;

  this->computeTimeOnTraj();
  
  for(unsigned int axis=0;  axis< traj.size(); axis++) {

   // Find segment Index
   idSeg = (traj[axis].size()) -1;
   while (time <= traj[axis][idSeg].timeOnTraj) { idSeg = idSeg - 1;}

     dt = time - traj[axis][idSeg].timeOnTraj;
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
  std::vector<double> durationArray;
  durationArray.clear();
  durationArray.resize(traj.size());
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
     durationArray[i] = traj[i][traj[i].size()-1].timeOnTraj + traj[i][traj[i].size()-1].time;
    } else {
     durationArray[i] = 0.0;
    }
    if(durationArray[i] > this->duration) {
      this->duration = durationArray[i];
    }
  }
  updateIC();
  return 0;
}

int SM_TRAJ::updateIC()
{

//  printf("updateIC\n");
  
  SM_STATUS resp;
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
  this->computeTimeOnTraj();
  cout.precision(4);
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl; 
  cout<<  "             TRAJECTORY               " << std::endl;  
  cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << std::endl; 
  cout << " Number of joints: " << traj.size() <<  std::endl;
  cout << " trajId: " << trajId <<  std::endl; 
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
  for(unsigned int j = 0; j < traj.size(); j++){
    std::cout << "===========  Traj on axis " << j <<" ==========" <<  std::endl; 
    std::cout << "   Number of segments: " << traj[j].size() <<  std::endl; 
    std::cout << "   Duration:           " << duration <<  std::endl; 
    std::cout << "   Segments:" <<  std::endl; 
    for(unsigned int k=0; k<traj[j].size();k++) {
      std::cout << std::fixed << " (" << k << "){Ti= "<< traj[j][k].timeOnTraj <<" }{T=" << traj[j][k].time << " , J=" <<  traj[j][k].jerk << "}{IC.x="<< traj[j][k].IC.x << "}" << std::endl;
    }
  }

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

int SM_TRAJ::load(char *name, int (*fct(void))) 
{
  ifstream file(name, ios::in);  // on ouvre en lecture
  std::vector<std::string> stringVector;
  int nbAxis = 0;
  //int nbSeg  = 0;
  string contenu;  // déclaration d'une chaîne qui contiendra la ligne lue
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
    printf("there are %d axes\n",nbAxis);
    /* Read qStart */
    doubleVector.clear();
    getline(file, contenu);
    doubleVector = parseFrame(contenu);
    for(unsigned int i=0; i <doubleVector.size(); i++) {
      qStart[i] = doubleVector[i];
    }
    printf("toto1\n");
    /* Read qGoal */
    doubleVector.clear();
    getline(file, contenu);
    doubleVector = parseFrame(contenu);
    for(unsigned int i=0; i <doubleVector.size(); i++) {
      qGoal[i] = doubleVector[i];
    }
   printf("toto2\n"); 
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
    cout << "SM_TRAJ::load file " << name << " loaded" << endl;  

  } else {
    std::cout << "Cannot open the file " << name << " !" << endl;
  }
  return 0;
}

int SM_TRAJ::convertToSM_TRAJ_STR(SM_TRAJ_STR *smTraj)
{
  //if(traj.size() != SM_TRAJ_NB_AXIS) {
  //  printf("ERROR nbAxis in traj : (%d) != SM_TRAJ_NB_AXIS\n", (int)traj.size());
  //  return 1;
  //}
  smTraj->trajId = this->trajId;
  smTraj->timePreserved = this->timePreserved;
  smTraj->nbAxis = (int)traj.size();

  for(int i=0; i<smTraj->nbAxis; i++) {
    smTraj->qStart[i] =  this->qStart[i];
  }
  for(int i=0; i<smTraj->nbAxis; i++) {
    smTraj->qGoal[i] =  this->qGoal[i];
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
  int nbSeg = (int)trajIn.size();
  int nbAxis = (int)trajIn[0].Time.size();


  //  printf("importFromSM_OUTPUT: There are %f axes and %f segments\n", (double)trajIn[0].Time.size(), (double)trajIn.size());

 
 this->resize(trajIn[0].Time.size());
 this->trajId = trajId;
 this->timePreserved = 0.0;
 
  for(int i=0; i<nbAxis; i++) {
    traj[i].resize(nbSeg);
    for(int j=0; j<nbSeg; j++) {
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
}

  /*set the initial condition in the first segment */
  for(int i=0; i<nbAxis; i++) {
      traj[i][0].IC.a	    =  trajIn[0].IC[i].a;
      traj[i][0].IC.v	    =  trajIn[0].IC[i].v;
      traj[i][0].IC.x	    =  trajIn[0].IC[i].x;
  }



 // compute the initial conditions and other variable for all segments
   computeTimeOnTraj();


  
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


int SM_TRAJ::approximate(std::vector< std::vector<SM_COND> > &trajIn, double timeStep, double errorPosMax,double errorVelMax, int id)
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

  printf("There are %d points and %d axes\n",nbSample, nbAxis);
  double total_time = nbSample*timeStep;

  for(int i=0; i< nbAxis; i++) {
    this->qStart[i] = trajIn[i][0].x;
  }
  for(int i=0; i< nbAxis; i++) {
    this->qGoal[i] = trajIn[i][nbSample-1].x;
  }

  Sm_Curve curv; // initialize and fill the ideal curve
  curv.traj.resize(nbSample);
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

  // utilise la classe SM_Approx pour effectuer l'approximation
  Sm_Approx approx;
  int res = approx.approximate(curv, timeStep, errorPosMax, errorVelMax, *this);

  return res;
}
