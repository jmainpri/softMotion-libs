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

  Fichier              : softMotionConst.h
  Fonction             : Constants definition for 3D Motion Trajectory Planning
  Date de creation     : Mai 2008
  Date de modification : Mai 2008
  Nb de lignes         :
  Auteur               : Xavier BROQUERE

----------------------------------------------------------------------*/

#define SM_NB_DIM 6
#define SM_NB_SEG 7
#define SM_DISTANCE_TOLERANCE_LINEAR       0.001
#define SM_DISTANCE_TOLERANCE_ANGULAR       0.005
#define SM_DISTANCE_TOLERANCE      0.001
#define SM_SAMPLING_TIME            0.010  // CAUTION Pay attention to the Motion task period */
#define SM_S_T                      10
#define SM_S_TD                     1000
#define SM_NB_TICK_SEC              100
