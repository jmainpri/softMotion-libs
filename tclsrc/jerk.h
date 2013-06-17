/*
#
# Copyright (c) 2010,2013 LAAS/CNRS
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

#ifndef JERK_H
#define JERK_H 


typedef struct jerkParams {
  double Jmax;
  double Amax;
  double Vmax;
} jerkParams;

typedef struct jerkData {
  double t1;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double dir;
  double dc;
  int zone;
} jerkData;

typedef struct jerkTimesAdjusted {
  double t1;
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double dir_a;
  double dir_b;
  double timp;
  double Vc;
} jerkTimesAdjusted ;

typedef struct jerkTimesAdjustedVc1Vc2 {
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
  double timp;
  double Vc1;
  double Vc2;
} jerkTimesAdjustedVc1Vc2;

typedef struct jerkConditions {
  double A;
  double V;
  double X;
} jerkConditions;

#ifdef __cplusplus
extern "C"
#endif
double jerkTimes(jerkParams* limits, jerkConditions* IC, jerkConditions* FC,
               jerkData *jerkdata);

#ifdef __cplusplus
extern "C"
#endif
double calculTimeProfileWithVcFixed(jerkParams* limits, jerkConditions* IC,
               jerkConditions* FC, jerkTimesAdjusted *jerkTimesAdjusted);

#ifdef __cplusplus
extern "C"
#endif
double adjustTimeSlowingVc1Vc2(jerkParams* limits, jerkConditions* IC,
               jerkConditions* FC,
               jerkTimesAdjustedVc1Vc2 *jerkTimesAdjustedVc1Vc2);

#endif
