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


#ifndef SOFT_MOTION_STRUCT_GENOM_H
#define SOFT_MOTION_STRUCT_GENOM_H



#define SM_TRAJ_NB_AXIS 32
#define SM_SEGMENT_TRACK_MAX_SIZE 128

/*
 * The two following structures are the new way 
 * to communicate between MHP and LWR
 */
typedef struct SM_SEGMENT_STR {
  int lpId;
  int unused;
  double timeOnTraj;
  double time;
  double ic_a;
  double ic_v;
  double ic_x;
  double jerk;
}SM_SEGMENT_STR;

typedef struct SM_TRAJ_AXIS_STR {
  int nbSeg;
  int unsused;
  SM_SEGMENT_STR seg[SM_SEGMENT_TRACK_MAX_SIZE];
} SM_TRAJ_AXIS_STR;

typedef struct SM_TRAJ_STR {
  int trajId;
  int nbAxis;
  double timePreserved;
  double qStart[SM_TRAJ_NB_AXIS];
  double qGoal[SM_TRAJ_NB_AXIS];
  SM_TRAJ_AXIS_STR traj[SM_TRAJ_NB_AXIS];
} SM_TRAJ_STR ;

#endif
