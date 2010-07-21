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


#ifndef SOFT_MOTION_STRUCT_H
#define SOFT_MOTION_STRUCT_H


#include <stdio.h>
#include <string.h>
#include <list>
#include <string>
#include <vector>



#include "softMotionConst.h"

/** @file softMotionStruct.h
 * @brief This file includes structure declarations for softMotion library.
 */

/**
 * @brief Status that return softMotion functions
 */
typedef enum SM_STATUS {
  SM_OK    = 0,
  SM_ERROR = 1
} SM_STATUS;

/**
 * @brief Enum ON/OFF
 */
typedef enum SM_SELECT {
	SM_ON  = 0,
	SM_OFF = 1
} SM_SELECT;  /* ON = 0, OFF = 1 */

/**
 * @brief Structure of the duration of seven named segments of softMotion
 */
typedef struct SM_TIMES {
    /** @brief Time of the Positive Jerk segment for the fisrt part of the motion (a)  */
	double Tjpa;
    /** @brief Time of the Constant Acceleration segment for the fisrt part of the motion (a)  */
	double Taca;
    /** @brief Time of the Negative Jerk segment for the fisrt part of the motion (a)  */
	double Tjna;
    /** @brief Time of the Constant Velocity segment */
	double Tvc;
    /** @brief Time of the Negative Jerk segment for the second part of the motion (b)  */
	double Tjnb;
    /** @brief Time of the Constant Acceleration segment for the second part of the motion (b)  */
	double Tacb;
	/** @brief Time of the Positve Jerk segment for the fisrt second of the motion (b)  */
	double Tjpb;
} SM_TIMES ;

/**
 * @brief Structure of the nseg segments of softMotion
 */
typedef struct SM_TIMES_DYN {
    /** @brief number of segments  */
	int nseg;
    /** @brief array of time of segments  */
	double* seg;
} SM_TIMES_DYN;

/**
 * @brief Structure of the kinematic constraints for one axis
 */
typedef struct SM_LIMITS {
  /** @brief Maximum jerk value (m/s^3) */
  double maxJerk;
  /** @brief Maximum acceleration value (m/s^2) */
  double maxAcc;
  /** @brief Maximum velocity value (m/s) */
  double maxVel;
} SM_LIMITS;

/**
 * @brief Structure of the kinematic constraints for two axis (cartesian and angular)
 */
typedef struct SM_POSELIMITS {
  /** @brief Linear constraints */
  SM_LIMITS linear;
  /** @brief Angular constraints */
  SM_LIMITS angular;
} SM_POSELIMITS;

/**
 * @brief Structure of the duration of seven unnamed segments of softMotion
 */
typedef struct SM_TIMES_GLOBAL {
  double T1;
  double T2;
  double T3;
  double T4;
  double T5;
  double T6;
  double T7;
} SM_TIMES_GLOBAL;

/**
 * @brief Structure of motion for a quaternion
 */
typedef struct SM_AXIS_TIMES {
  SM_TIMES PX;
  SM_TIMES PY;
  SM_TIMES PZ;
  SM_TIMES QN;
  SM_TIMES QI;
  SM_TIMES QJ;
  SM_TIMES QK;
} SM_AXIS_TIMES;

/**
 * @brief Structure of kinematic conditions for one axis
 */
typedef struct SM_COND {
  /** @brief Acceleration */
  double a;
  /** @brief Velocity */
  double v;
  /** @brief Position */
  double x;
} SM_COND;

/**
 * @brief Structure of the 4 jerk values for a point to point motion
 */
typedef struct SM_JERKS {
  /** @brief select the type of notion
  *
  * Sel = 1 -->  J1=J2=J3=J4
  * Sel = 4 -->  J1, J2, J3, J4
  */
  int sel;
  /** @brief jerk value of the segement 1 */
  double J1;
  /** @brief jerk value of the segement 3 */
  double J2;
  /** @brief jerk value of the segement 5 */
  double J3;
  /** @brief jerk value of the segement 7 */
  double J4;
  /** @brief i forgot  */
  int withOutUseAlignment;
} SM_JERKS;

/**
 * @brief Structure of the particular velocities used to find the critical length and the type of motion
 */
typedef struct SM_PARTICULAR_VELOCITY {
  /** @brief Velocity of the start point at maximun acceleration and with a lower velocity (minus) than the start point */
  double Vsmm;
  /** @brief Velocity of the start point at maximun acceleration and with a greater velocity (plus) than the start point */
  double Vsmp;
  /** @brief Velocity of the start point at zero acceleration and with a lower velocity (minus) than the start point */
  double Vs0m;
  /** @brief Velocity of the start point at zero acceleration and with a greater velocity (plus) than the start point */
  double Vs0p;
  /** @brief Velocity of the final point at maximum acceleration and with a lower velocity (minus) than the final point */
  double Vfmm;
  /** @brief Velocity of the final point at maximum acceleration and with a greater velocity (plus) than the final point */
  double Vfmp;
  /** @brief Velocity of the final point at zero acceleration and with a lower velocity (minus) than the final point */
  double Vf0m;
  /** @brief Velocity of the final point at zero acceleration and with a greater velocity (plus) than the final point */
  double Vf0p;
  /** @brief Velocity value reached at zero acceleration on the jerk positive parabola passing through the point with a maximun acceleration/velocity */
  double Vlim;
} SM_PARTICULAR_VELOCITY;

/**
 * @brief Structure for a complete softMotion for SM_NB_DIM axes
 */
typedef struct SM_MOTION {
  SM_TIMES TNE; /* use this only for point to point motion in xarm */
  SM_TIMES Times[SM_NB_DIM]; /* en seconde */
  SM_TIMES TimesM[SM_NB_DIM];
  SM_TIMES Acc[SM_NB_DIM];
  SM_TIMES Vel[SM_NB_DIM];
  SM_TIMES Pos[SM_NB_DIM];
  SM_JERKS jerk[SM_NB_DIM];
  SM_COND  IC[SM_NB_DIM];
  SM_COND  FC[SM_NB_DIM];
  int      Dir[SM_NB_DIM];
  int      Dir_a[SM_NB_DIM];
  int      Dir_b[SM_NB_DIM];
  double   MotionDuration[SM_NB_DIM]; // motion duration of each axis
 	double      MotionDurationM[SM_NB_DIM];
	double TimeCumul[SM_NB_DIM][SM_NB_SEG];
  int      TimeCumulM[SM_NB_DIM][SM_NB_SEG];
	int      motionIsAdjusted[SM_NB_DIM];
} SM_MOTION;

typedef struct SM_MOTION_DYN {
	SM_TIMES* TNE;
	SM_TIMES* Times; /* en seconde */
	SM_TIMES* TimesM;
	SM_TIMES* Acc;
	SM_TIMES* Vel;
	SM_TIMES* Pos;
	SM_JERKS* jerk;
	SM_COND*  IC;
	SM_COND*  FC;
	double*  Jerk;
	int*     Dir;
	double*   MotionDuration; // motion duration of each axis
	double*   MotionDurationM;
	int*     TimeCumulM;
	int*      motionIsAdjusted;
} SM_MOTION_DYN;

typedef struct SM_MOTION_MONO {
	SM_TIMES Times; /* en seconde */
	SM_TIMES TimesM;
	SM_TIMES Acc;
	SM_TIMES Vel;
	SM_TIMES Pos;
	SM_JERKS jerk;
	SM_COND  IC;
	SM_COND  FC;
	double   Jerk;
	int      Dir;
	int      Dir_a;
	int      Dir_b;
	double   MotionDuration;
	double   MotionDurationM;
	double   TimeCumul[SM_NB_SEG];
	int      TimeCumulM[SM_NB_SEG];
	int      motionIsAdjusted;
} SM_MOTION_MONO;

typedef struct SM_TRANSITION_MOTION {
  SM_TIMES Times[SM_NB_DIM]; /* en seconde */
  SM_TIMES TimesM[SM_NB_DIM];
  SM_TIMES Acc_a[SM_NB_DIM];
  SM_TIMES Vel_a[SM_NB_DIM];
  SM_TIMES Pos_a[SM_NB_DIM];
  SM_JERKS jerk[SM_NB_DIM];
  SM_TIMES Acc_b[SM_NB_DIM];
  SM_TIMES Vel_b[SM_NB_DIM];
  SM_TIMES Pos_b[SM_NB_DIM];
  SM_TIMES Acc[SM_NB_DIM];
  SM_TIMES Vel[SM_NB_DIM];
  SM_TIMES Pos[SM_NB_DIM];
  int      Dir[SM_NB_DIM];
  SM_COND  IC[SM_NB_DIM];
  SM_COND  FC[SM_NB_DIM];
  int      Dir_a[SM_NB_DIM];
  int      Dir_b[SM_NB_DIM];
  int      MotionDuration[SM_NB_DIM];
  int      optimalTime[SM_NB_DIM]; /* en ticks */
  int      timeToStop;
} SM_TRANSITION_MOTION;

typedef struct SM_TIMES_ADJUSTED_MOTION {
  double Tvc0;
  double Tjpa;
  double Taca;
  double Tjna;
  double Tvc1;
  double Tjnb;
  double Tacb;
  double Tjpb;
  double Tvc2;
  double Tjpc;
  double Tacc;
  double Tjnc;
} SM_TIMES_ADJUSTED_MOTION;

typedef struct SM_SEGMENT {
#include "softMotionStruct.h"
	int type; /* 1 to 7 */
	double time;
	int timeM;
	double J;
	double A0;
	double V0;
	double X0;
	int dir;
} SM_SEGMENT;

typedef enum SM_INTERP_TYPE {
	SM_PASS_PTP  = 0,
	SM_PASS_BY   = 1,
	SM_PASS_NEAR = 2
} SM_INTERP_TYPE;

typedef struct SM_TIMES_STOP {
	double T1;
	double T2;
	double T3;
	double T4;
	double T5;
	double T6;
	double T7;
	double T8;
	double T9;
	double T10;
} SM_TIMES_STOP;

/*--------------------------- Trajectory defenition ---------------------*/
typedef struct SM_LINE_ARC{
	double invR0; // inverse of starting radius
	double invRf; // inverse of ending radius
	double Ls;    // length of line or arc
} SM_LINE_ARC;

/**
 * @brief Structure of datas for trajectory
 */
typedef struct SM_CURVE_DATA{
    /** @brief time value of the given data*/
	double t;
    /** @brief curvature abcissa*/
	double u;
    /** @brief tangential velocity*/
	double du;
    /** @brief tangential acceleration*/
	double ddu;
    /** @brief cartesian coordinate*/
	double Pos[3];
    /** @brief projected velocity in cartesian coordinate*/
	double Vel[3];
    /** @brief projected acceleration in cartesian coordinate*/
	double Acc[3];
    /** @brief projected Jerk in cartesian coordinate*/
    double Jerk[3];
    /** @brief acceleration norm*/
	double AccNorm;
    /** @brief length of trajectory*/
	double absci;
}SM_CURVE_DATA;

typedef struct SM_ROT{
	double thetaX; // angle of rotation /x
	double thetaY; // angle of rotation /y
	double thetaZ; // angle of rotation /z
	double R[3][3]; //rotation matrix
}SM_ROT;

/**
 * @brief Structure of dimension of conditions
 */
typedef struct SM_COND_DIM{
    /** @brief Initial or final condition in 3 axis*/
	SM_COND Axis[3];
} SM_COND_DIM;

/**
 * @brief Structure of output
 */
typedef struct SM_OUTPUT{
    /** @brief output jerk for n axes*/
  std::vector<double> Jerk;
  /** @brief output time for n axes*/
  std::vector<double> Time;
  /** @brief output initial condition (at the beginning of the segmnent, before applying Jerk for n axes*/
  std::vector<SM_COND> IC;
}SM_OUTPUT;


// #ifdef __cplusplus
/**
 * @brief Structure of two dimentional point
 */
typedef struct Point2D
{
  /** @brief coordinate in axis X and Y*/
  double x, y;
} Point2D;

/**
 * @brief Structure of Sub-Curve type
 */
typedef enum SubPathType
  {
    /** @brief type of line */
    LINE,
    /** @brief type of theoretic line */
    LINE_TH,
    /** @brief type of bezier curve */
    BEZIER3,
    /** @brief type of circle */
    CERCLE,
    /** @brief type of sinusoid */
    SINUS,
    /** @brief type of parabol */
    PARABOL,
  } SubPathType;

/**
 * @brief Structure of Sinusoid
 */
typedef struct SinusParams
{
    /** @brief initial point of a sinusoid */
    Point2D start;
    /** @brief frequency of a sinusoid as in y = a*sin(2*PI*f*t + phi)*/
    double frequency;
    /** @brief amplitude of a sinusoid as in y = a*sin(2*PI*f*t + phi)*/
    double amplitude;
    /** @brief phase of a sinusoid as in y = a*sin(2*PI*f*t + phi)*/
    double phase;
    /** @brief length of a sinusoid which is projected in axis X*/
    double length_x;
} SinusParams ;

/**
 * @brief Structure of Circle
 */
typedef struct CercleParams
{
    /** @brief center of a circle*/
    Point2D center;
    /** @brief radius of a circle*/
    double radius;
    /** @brief parameters of circle as in x = a*cos(2*PI*f*t); y = a*sin(2*PI*f*t) */
    SinusParams sinus_para;
} CercleParams ;

/**
 * @brief Structure of lines
 */
typedef struct LineParams
{
  /** @brief initial and final points of a line*/
  Point2D start, end;
}LineParams;

/**
 * @brief Structure of Parabol
 */
typedef struct ParabolParams
{
  /** @brief parameter 'a' as in y = ax^2 */
  double a;
  /** @brief initial coordinate in axis X */
  double start_x;
  /** @brief final coordinate in axis X */
  double end_x;
}ParabolParams;

/**
 * @brief Structure of Sub-Curves
 */
typedef struct SubPath
{
  /** @brief curve type */
  SubPathType type;
  /** @brief initial and final points of the curve */
  Point2D start, end;
  /** @brief two control points in the middle*/
  Point2D bezier3[2];
  /** @brief theoretic circle */
  CercleParams cercle;
  /** @brief theoretic sinusoid */
  SinusParams sinus;
  /** @brief theoretic line*/
  LineParams line;
  /** @brief theoretic parabol */
  ParabolParams parabol;
} SubPath;

/**
 * @brief Structure of Curves
 */
typedef struct Path
{
  /** @brief 2-dimension initial point of curve */
  Point2D origin;
  /** @brief number of sub-curve */
  int nbSubPath;
  /** @brief length of curves */
  double length;
  /** @brief list for the sub-curves */
  std::list<SubPath> subpath;
} Path;

/**
 * @brief Structure of points in Viewer
 */
typedef struct kinPoint
{
  /** @brief coordinate of points */
  SM_COND kc[3];
  /** @brief time of points */
  double t;
} kinPoint;

/**
 * @brief Structure of sub-trajectory
 */
typedef struct SubTraj
{
  /** @brief motion data for the sub-trajectory */
  std::vector<SM_CURVE_DATA> traj;
  /** @brief jerk value, interval time and conditions for three segments */
  std::vector<SM_OUTPUT> motion_par_seg;
  /** @brief initial condition for three segments */
  std::vector<SM_COND_DIM> IC_par_seg;
  /** @brief final condition for three segments */
  std::vector<SM_COND_DIM> FC_par_seg;
  /** @brief error beteween the sub-trajectories */
  double err;
  int flag_traj;
  int point_depart;
} SubTraj;

/**
 * @brief Structure of index for the point
 */
typedef struct IndiceTrace
{
  /** @brief discretised length of a curve */
  double length_accumul;
  /** @brief index of axis X for a point in the curve */
  double x;
  /** @brief index of axis Y for a point in the curve */
  double y;
}IndiceTrace;

typedef struct Traj
{
  std::vector<SM_CURVE_DATA> traj;

} Traj;

typedef struct TrajVec
{
  std::vector<Traj> trajvec;
} TrajVec;
// #endif


#endif










