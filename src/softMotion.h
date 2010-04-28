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


/**
 ** softMotion.h
 **
 ** Date: Mai 2008
 **
 **/


#ifndef SOFTMOTION_H
#define SOFTMOTION_H


#include "softMotionStruct.h"


#ifndef ABS
#define ABS(a)        (((a) < 0) ? (-(a)) : (a))
#endif

#ifdef __cplusplus
#include <string>
#include <list>
#include <math.h>
#include <vector>
#endif

SM_STATUS sm_VerifyInitialAndFinalConditions(SM_LIMITS* limitsGoto, SM_COND* IC, SM_COND* FC, SM_PARTICULAR_VELOCITY* PartVel, SM_COND* ICm, SM_COND* FCm);

extern SM_STATUS sm_CalculOfCriticalLengthLocal( SM_LIMITS* limitsGoto, SM_PARTICULAR_VELOCITY* PartVel, SM_COND* IC, SM_COND* FC, double* dc, int* zone);

/************************************************/
/*             Functions of first type          */
/************************************************/

SM_STATUS sm_CalculOfDSVmaxType1(SM_LIMITS* limitsGoto, SM_PARTICULAR_VELOCITY* PartVel, SM_COND* ICm, SM_COND* FCm, double* DSVmax);

SM_STATUS sm_CalculOfDSAmaxType1(SM_LIMITS* limitsGoto, SM_PARTICULAR_VELOCITY* PartVel, SM_COND* ICm, SM_COND* FCm, double* DSAmax);

SM_STATUS sm_VerifyTimesType1(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm,  SM_TIMES* Time, double epsilon_erreur);

SM_STATUS sm_JerkProfile_Type1_VmaxAmin_7seg_maxi(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel,  SM_TIMES* Time);

SM_STATUS sm_JerkProfile_Type1_VmaxAmin_5seg_maxi(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel,  SM_TIMES* Time);

SM_STATUS sm_JerkProfile_Type1_VmaxAmin_6seg_maxi(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel,  SM_TIMES* Time);

SM_STATUS sm_JerkProfile_Type1_Vmax_5seg_maxi(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel,  SM_TIMES* Time);

SM_STATUS sm_JerkProfile_Type1_Vmax_6seg_maxi(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel,  SM_TIMES* Time);

SM_STATUS sm_JerkProfile_Type1_VmaxAmin_4seg(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel,  SM_TIMES* Time);

SM_STATUS sm_JerkProfile_Type1_inf_DSAmax_Z0(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm,  SM_TIMES* Time);

SM_STATUS sm_JerkProfile_Type1_inf_DSAmax_Z1(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel,  SM_TIMES* Time, double* dc);

SM_STATUS sm_JerkProfile_Type1_inf_DSAmax_Z2(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel,  SM_TIMES* Time, double* dc);

SM_STATUS sm_Calcul_Of_DSAmax2_2_Type1(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel, double* DSAmaxT12_2);

SM_STATUS sm_Calcul_Of_DSAmax2_22_Type1(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel, double *DSAmaxT12_22);

SM_STATUS sm_JerkProfile_Type1_inf_DSAmax_inf_DSAmaxT12_22_Z2(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel,  SM_TIMES* Time);

SM_STATUS sm_JerkProfile_Type1_inf_DSAmax_Z3(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm,  SM_TIMES* Time);

SM_STATUS sm_Calcul_Of_DSAmax2_31_Type1(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel, double* DSAmaxT12_31);

SM_STATUS sm_Calcul_Of_DSAmax2_32_Type1(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel, double* DSAmaxT12_32);

SM_STATUS sm_Calcul_Of_DSAmax2_33_Type1(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, double* DSAmaxT12_33);

SM_STATUS sm_Calcul_Of_DSAmax2_4_Type1(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel, double *dc, double* DSAmaxT12_4);

SM_STATUS sm_JerkProfile_Type1_inf_DSAmax_sup_DSAmax2_31_Z3(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel,  SM_TIMES* Time);

SM_STATUS sm_JerkProfile_Type1_inf_DSAmax_sup_DSAmax2_32_Z3(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel,  SM_TIMES* Time);

SM_STATUS sm_JerkProfile_Type1_inf_DSAmax_sup_DSAmax2_33_Z3(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm,   SM_TIMES* Time);

SM_STATUS sm_Jerk_Profile_dc(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm, SM_PARTICULAR_VELOCITY* PartVel, int *zone,  SM_TIMES* Time);

SM_STATUS sm_VerifyTimesType2(SM_LIMITS* limitsGoto, SM_COND* ICm, SM_COND* FCm,  SM_TIMES* Time);

/*************************************************/
/*                  Main Functions               */
/*************************************************/

extern SM_STATUS sm_ComputeSoftMotion(SM_COND IC, SM_COND FC, SM_LIMITS limitsGoto, SM_TIMES *T_Jerk, int *TrajectoryType);

extern SM_STATUS sm_ComputeSoftMotionLocal(SM_COND IC, SM_COND FC, SM_LIMITS limitsGoto, SM_TIMES *T_Jerk, int *TrajectoryType, double* dcOut, int* zoneOut);

extern SM_STATUS sm_CalculOfCriticalLength(SM_COND IC, SM_COND FC, SM_LIMITS limitsGoto, double* dc);

extern SM_STATUS sm_AdjustTimeSlowingJerk(SM_COND IC, SM_COND FC,double MaxTime, SM_LIMITS Limits, SM_TIMES *Time, double* Jerk, SM_LIMITS* newLimits, int* DirTransition);

extern SM_STATUS sm_CalculTimeProfileWithVcFixed(double V0, double Vf, double Vc, SM_LIMITS Limits, SM_TIMES_GLOBAL *Time, int* dir_a, int* dir_b);

extern SM_STATUS sm_CalculTimeProfileWithVcFixedWithAcc(double V0, double Vf, double A0, double Af, double Vc, SM_LIMITS Limits, SM_TIMES_GLOBAL *Time, int* dir_a, int* dir_b);

extern SM_STATUS sm_CalculOfDistance( double V0, double A0, double Jmax, SM_TIMES_GLOBAL* Time, int* dir_a, int* dir_b, double* FinalDist);

extern SM_STATUS sm_AdjustTimeSlowingVelocity(double V0, double Vf, double GoalDist, SM_LIMITS Limits, double TimeImp, SM_TIMES *Time, int* dir_a, int* dir_b);

extern SM_STATUS sm_AdjustTimeSlowingVelocityWithAcc(double V0, double Vf,double A0, double Af, double GoalDist, SM_LIMITS Limits, double TimeImp, SM_TIMES *Time, int* dir_a, int* dir_b);

extern SM_STATUS sm_AdjustTime(SM_COND IC, SM_COND FC,double MaxTime, SM_LIMITS Limits, SM_TIMES *Time, double* Jerk, int* DirTransition_a, int* DirTransition_b);

extern SM_STATUS sm_AdjustTimeWithAcc(SM_COND IC, SM_COND FC,double MaxTime, SM_LIMITS Limits, SM_TIMES *Time, double* Jerk, int* DirTransition_a, int* DirTransition_b);

extern SM_STATUS sm_VerifyTimes_Dir_ab(double dist_error, double GD, SM_JERKS Jerks, SM_COND IC, int Dir_a, int Dir_b, SM_TIMES Time, SM_COND *FC, SM_TIMES *Acc, SM_TIMES *Vel, SM_TIMES *Pos);

extern SM_STATUS sm_VerifyTimes(double dist_error, double GD, SM_JERKS Jerks, SM_COND IC, int Dir, SM_TIMES Time, SM_COND *FC, SM_TIMES *Acc, SM_TIMES *Vel, SM_TIMES *Pos, SM_SELECT msgError);
//extern SM_STATUS findTransitionTime(SM_COND IC[XARM_NB_DIM], SM_COND FC[XARM_NB_DIM], SM_POSELIMITS limitsGoto, int stopTimeM, int optimalTimeM[XARM_NB_DIM], int *impTimeM, SM_TIMES Times[XARM_NB_DIM], int Dir_a[XARM_NB_DIM], int Dir_b[XARM_NB_DIM]);

extern SM_STATUS sm_FindTransitionTime( SM_POSELIMITS limitsGoto, SM_TRANSITION_MOTION* motion, int *impTimeM);

extern SM_STATUS sm_CalculOfAccVelPosAtTime(int tick, SM_SEGMENT* seg, SM_COND* v);

extern SM_STATUS sm_CalculOfAccVelPosAtTimeSecond(double t, SM_SEGMENT* seg, SM_COND* v);

extern SM_STATUS sm_SamplingAdjustTime(double Time, double *aTime);
extern SM_STATUS sm_GetMonotonicTimes(SM_TIMES Times,SM_TIMES *TM, int *NOE);
extern SM_STATUS sm_GetNumberOfElement (SM_TIMES* TM, SM_TIMES* TNE);
extern void sm_SM_TIMES_copy_into(const SM_TIMES *e, SM_TIMES* s);
extern void sm_sum_motionTimes(SM_TIMES* sm_times, double* sum);
extern void sm_copy_SM_MOTION_into(const SM_MOTION* e, SM_MOTION* s);
extern void sm_copy_SM_MOTION_MONO_into(const SM_MOTION_MONO* e, SM_MOTION_MONO* s);
extern SM_STATUS sm_ComputeSoftMotionPointToPoint(SM_COND IC[SM_NB_DIM], SM_COND FC[SM_NB_DIM], SM_POSELIMITS Limits, SM_MOTION *motion);
extern SM_STATUS sm_ComputeSoftMotionPointToPoint_gen(int nbAxis,double* J_max, double *A_max, double *V_max, SM_MOTION_MONO *motion);

extern SM_STATUS GeometricalCalculationsPointToPoint(double PFrom[SM_NB_DIM], double PTo[SM_NB_DIM],	double *MaxLDist,  double *MaxADist, double Dist[SM_NB_DIM], double AbsDist[SM_NB_DIM], int Dir[SM_NB_DIM]) ;

extern SM_STATUS sm_CalculPointToPointJerkProfile(double PFrom[SM_NB_DIM], SM_POSELIMITS Limits, double MaxLDist, double MaxADist, double MaxDist, double AbsDist[SM_NB_DIM], double Dir[SM_NB_DIM], SM_JERKS jerk[], SM_TIMES Acc[], SM_TIMES Vel[], SM_TIMES Pos[], SM_TIMES* TNE);

extern SM_STATUS sm_adjustMotionWith3seg( SM_COND IC, SM_COND FC, double Timp, SM_MOTION_MONO *motion);

extern SM_STATUS sm_AVX_TimeVar(double IC[3], double *T, double *J, int nbSeg, double *t, int nbSample, double *a, double *v, double *x);
//===================
#ifdef __cplusplus
//extern SM_STATUS sm_ComputeCondition(std::vector<SM_CURVE_DATA> &IdealTraj, std::vector<SM_COND_DIM> &IC, std::vector<SM_COND_DIM> &FC, std::vector<double> &Timp, std::vector<int> &IntervIndex);
//extern SM_STATUS sm_SolveWithoutOpt(std::vector<SM_COND_DIM> &IC, std::vector<SM_COND_DIM> &FC, std::vector<double> &Timp, std::vector<SM_OUTPUT> &motion);
extern SM_STATUS parseSvg(std::string fileName, std::list<Path> &path, double* width, double* height);
extern SM_STATUS constructTrajSvg(std::list<Path> &path, double tic, SM_LIMITS Lim, std::vector<SM_CURVE_DATA> &IdealTraj);
extern SM_STATUS plotIdealTraj(std::string fileName, std::vector<SM_CURVE_DATA> &IdealTraj, double width, double height);
extern SM_STATUS plotApproxTraj(std::string fileName, std::vector<SM_CURVE_DATA> &IdealTraj, double width, double height);
extern SM_STATUS convertMotionToCurve(std::vector<SM_OUTPUT> &motion, double tic,double nbIntervals,  std::vector<SM_CURVE_DATA>  &ApproxTraj);
extern void wait_for_key ();
extern SM_STATUS saveTraj(std::string fileName, std::vector<SM_CURVE_DATA> &traj);

extern SM_STATUS sm_ComputeCondition(std::vector<SM_CURVE_DATA> &IdealTraj, std::vector<kinPoint> &discPoint, std::vector<SM_COND_DIM> &IC, std::vector<SM_COND_DIM> &FC, std::vector<double> &Timp, std::vector<int> &IntervIndex);
extern SM_STATUS sm_SolveWithoutOpt(std::vector<SM_COND_DIM> &IC,std::vector<SM_COND_DIM> &FC, std::vector<double> &Timp, std::vector<SM_OUTPUT> &motion);

// Functions from Wang

extern SM_STATUS Vel_Profile(std::vector<SM_CURVE_DATA>  &IdealTraj, std::vector<double> &vel_discr_X,std::vector<double> &vel_discr_Y, std::vector<double> &acc_discr_X, std::vector<double> &acc_discr_Y);
extern SM_STATUS Vel_Profile_Path(std::list<Path> &path, std::vector<double> &vel_path_x, std::vector<double> &vel_path_y, double  sample_time);
extern SM_STATUS Courbure(std::list<Path> &path, std::vector<double> &curvature);
extern SM_STATUS Calcul_Error(std::vector<SM_CURVE_DATA>  &IdealTraj, std::vector<SM_CURVE_DATA> &ApproxTraj, kinPoint *errorMax, std::vector<double> & error);
#endif
#endif
