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


#include "jerk.h"
#include "../src/softMotion.h"
#include "../src/softMotionStruct.h"
#include <stdio.h>

double jerkTimes(jerkParams* limits, jerkConditions* IC, jerkConditions* FC, jerkData *jerkdata) {
  SM_LIMITS limitsGoto;
  SM_COND IC2;
  SM_COND FC2;
  SM_TIMES T_Jerk;
  int Dir;
  double dc;
  int zone;

  /*Set Limits*/
  limitsGoto.maxVel = limits->Vmax;
  limitsGoto.maxAcc = limits->Amax;
  limitsGoto.maxJerk = limits->Jmax;

  /* set initial and final conditions */
  IC2.a = IC->A;
  IC2.v = IC->V;
  IC2.x = IC->X;
  FC2.a = FC->A;
  FC2.v = FC->V;
  FC2.x = FC->X;

  if (sm_ComputeSoftMotionLocal(IC2, FC2, limitsGoto, &T_Jerk, &Dir, &dc, &zone)!= 0) {
    printf("ERROR Jerk Profile\n");
  }

  jerkdata->dc = dc;
  jerkdata->dir = Dir;
  jerkdata->zone = zone;

  jerkdata->t1 = T_Jerk.Tjpa;
  jerkdata->t2 = T_Jerk.Taca;
  jerkdata->t3 = T_Jerk.Tjna;
  jerkdata->t4 = T_Jerk.Tvc;
  jerkdata->t5 = T_Jerk.Tjnb;
  jerkdata->t6 = T_Jerk.Tacb;
  jerkdata->t7 = T_Jerk.Tjpb;

  return 1;
}


double adjustTimeSlowingVc1Vc2(jerkParams* limits, jerkConditions* IC, jerkConditions* FC, jerkTimesAdjustedVc1Vc2 *jerkTimesAdjustedVc1Vc2)
{
  double V0, Vf, A0, Af, GoalDist;
  SM_LIMITS Limits;
  SM_TIMES_ADJUSTED_MOTION T;
  int dir_a, dir_b;

  /*Set Limits*/
  Limits.maxVel = limits->Vmax;
  Limits.maxAcc = limits->Amax;
  Limits.maxJerk = limits->Jmax;

  /* set initial and final conditions */
  V0 = IC->V;
  Vf = FC->V;
  A0 = IC->A;
  Af = FC->A;

  GoalDist =  FC->X - IC->X;

  // xarmAdjustimeWithVc1AndVc2( V0, GoalDist, jerkTimesAdjustedVc1Vc2->timp, Limits, &T);

  jerkTimesAdjustedVc1Vc2->Tjpa = T.Tjpa;
  jerkTimesAdjustedVc1Vc2->Taca = T.Taca;
  jerkTimesAdjustedVc1Vc2->Tjna = T.Tjna;
  jerkTimesAdjustedVc1Vc2->Tvc1 = T.Tvc1;
  jerkTimesAdjustedVc1Vc2->Tjnb = T.Tjnb;
  jerkTimesAdjustedVc1Vc2->Tacb = T.Tacb;
  jerkTimesAdjustedVc1Vc2->Tjpb = T.Tjpb;
  jerkTimesAdjustedVc1Vc2->Tvc2 = T.Tvc2;
  jerkTimesAdjustedVc1Vc2->Tjpc = T.Tjpc;
  jerkTimesAdjustedVc1Vc2->Tacc = T.Tacc;
  jerkTimesAdjustedVc1Vc2->Tjnc = T.Tjnc;

  return 1;
}


double calculTimeProfileWithVcFixed(jerkParams* limits, jerkConditions* IC, jerkConditions* FC, jerkTimesAdjusted *jerkTimesAdjusted)
{
  double V0, Vf, A0, Af;
  SM_LIMITS Limits;
  SM_TIMES_GLOBAL T;
  int dir_a, dir_b;
  double FinalDist;

  /*Set Limits*/
  Limits.maxVel = limits->Vmax;
  Limits.maxAcc = limits->Amax;
  Limits.maxJerk = limits->Jmax;

  /* set initial and final conditions */
  V0 = IC->V;
  Vf = FC->V;
  A0 = IC->A;
  Af = FC->A;

  sm_CalculTimeProfileWithVcFixedWithAcc( V0, Vf,A0, Af, jerkTimesAdjusted->Vc, Limits, &T, &dir_a, &dir_b);

  T.T4 = jerkTimesAdjusted->timp - (T.T1 + T.T2 + T.T3 + T.T5 + T.T6 + T.T7);

  if ( T.T4 < 0.0) {  T.T4 = 0.0;}
  T.T4 = 0.0;
  sm_CalculOfDistance( V0, A0, Limits.maxJerk, &T, &dir_a, &dir_b, &FinalDist);
  printf("Distance Parcourue %f\n",FinalDist);
  jerkTimesAdjusted->t1 = T.T1;
  jerkTimesAdjusted->t2 = T.T2;
  jerkTimesAdjusted->t3 = T.T3;
  jerkTimesAdjusted->t4 = T.T4;
  jerkTimesAdjusted->t5 = T.T5;
  jerkTimesAdjusted->t6 = T.T6;
  jerkTimesAdjusted->t7 = T.T7;
  jerkTimesAdjusted->dir_a = dir_a;
  jerkTimesAdjusted->dir_b = dir_b;


  return 1;
}


