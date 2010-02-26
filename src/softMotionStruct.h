/*--------------------------------------------------------------------
                         -- C N R S --
         Laboratoire d'automatique et d'analyse des systemes
            Groupe Robotique et Intelligenxce Artificielle
                   7 Avenue du colonel Roche
                     31 077 Toulouse Cedex

  Fichier              : softMotionStruct.h
  Fonction             : Structures definition for 3D Motion Trajectory Planning
  Date de creation     : Mai 2008
  Date de modification : Mai 2008
  Nb de lignes         :

  Auteur               : Xavier BROQUERE

----------------------------------------------------------------------*/

#ifndef SOFT_MOTION_STRUCT_H
#define SOFT_MOTION_STRUCT_H



#include "softMotionConst.h"

typedef enum SM_STATUS {
  SM_OK    = 0,
  SM_ERROR = 1
} SM_STATUS;

typedef enum SM_SELECT {
	SM_ON  = 0,
	SM_OFF = 1
} SM_SELECT;  /* ON = 0, OFF = 1 */

typedef struct SM_TIMES {
	double Tjpa;
	double Taca;
	double Tjna;
	double Tvc;
	double Tjnb;
	double Tacb;
	double Tjpb;
} SM_TIMES ;

typedef struct SM_TIMES_DYN {
	int nseg;
	double* seg;
} SM_TIMES_DYN;

typedef struct SM_LIMITS {
  double maxJerk;
  double maxAcc;
  double maxVel;
} SM_LIMITS;

typedef struct SM_POSELIMITS {
  SM_LIMITS linear;
  SM_LIMITS angular;
} SM_POSELIMITS;

typedef struct SM_TIMES_GLOBAL {
  double T1;
  double T2;
  double T3;
  double T4;
  double T5;
  double T6;
  double T7;
} SM_TIMES_GLOBAL;

typedef struct SM_AXIS_TIMES {
  SM_TIMES PX;
  SM_TIMES PY;
  SM_TIMES PZ;
  SM_TIMES QN;
  SM_TIMES QI;
  SM_TIMES QJ;
  SM_TIMES QK;
} SM_AXIS_TIMES;

typedef struct SM_COND {
  double a;             /* Acceleration */
  double v;             /* Velocity     */
  double x;             /* Distance     */
} SM_COND;

typedef struct SM_JERKS {
  int sel;              /* Sel = 1   J1=J2=J3=J4    */
                        /* Sel = 4   J1, J2, J3, J4 */
  double J1;
  double J2;
  double J3;
  double J4;
  int withOutUseAlignment;
} SM_JERKS;

typedef struct SM_PARTICULAR_VELOCITY {
  double Vsmm;
  double Vsmp;
  double Vs0m;
  double Vs0p;
  double Vfmm;
  double Vfmp;
  double Vf0m;
  double Vf0p;
  double Vlim;
} SM_PARTICULAR_VELOCITY;

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



#endif
