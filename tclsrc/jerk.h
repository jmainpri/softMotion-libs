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

extern double jerkTimes(jerkParams* limits, jerkConditions* IC, jerkConditions* FC, jerkData *jerkdata);

//extern double adjustTimeSlowingVc(jerkParams* limits, jerkConditions* IC, jerkConditions* FC, jerkTimesAdjusted *jerkTimesAdjusted);

extern double calculTimeProfileWithVcFixed(jerkParams* limits, jerkConditions* IC, jerkConditions* FC, jerkTimesAdjusted *jerkTimesAdjusted);

extern double adjustTimeSlowingVc1Vc2(jerkParams* limits, jerkConditions* IC, jerkConditions* FC, jerkTimesAdjustedVc1Vc2 *jerkTimesAdjustedVc1Vc2);

#endif
