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
#define uint unsigned int
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
  this->timePreserved = traj.timePreserved;
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
  tsVec.clear();
  duration = 0.0;
  timePreserved = 0.0;;
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
   while (time < traj[axis][idSeg].timeOnTraj) { idSeg = idSeg - 1;}
   if(idSeg <0) {
     printf("ERROR Big up, not possible!!\n");
   }
   
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
  //this->computeTimeOnTraj();
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
      std::cout << std::fixed << " (" << k << "){Ti= "<< traj[j][k].timeOnTraj <<" }{T=" << traj[j][k].time << " , J=" <<  traj[j][k].jerk << "}{IC.x="<< traj[j][k].IC.x << "}" <<  "}{IC.v="<< traj[j][k].IC.v << "}" << "}{IC.a="<< traj[j][k].IC.a << "}" << std::endl;
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

  this->clear();
  this->resize(1);
  traj.clear();
 
  std::vector<double> I(3);
  std::vector<double> T(SM_NB_SEG);
  std::vector<double> J(SM_NB_SEG);

  /* compute the motion */
  resp = sm_ComputeSoftMotion(IC, FC, limits, &T_Jerk, &dir);
  if (resp != SM_OK) {
    printf("ERROR sm_ComputeSoftMotion\n");
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

  computeTimeOnTraj();
  return 0;
}


//int SM_TRAJ::computePTPmotion(std::vector< double > start, std::vector< double > goal, std::vector< double > jmax,  std::vector< double > amax,  std::vector< double > vmax)
