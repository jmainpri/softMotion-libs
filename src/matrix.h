/* matrix.h 111.2 4/24/87 */

/* matrix.h -- define types for matrices using Iliffe vectors
 *
 * This version has been customized for use with the Generalized
 * Image Library.
 *************************************************************
 * HISTORY
 * 28-Feb-87  Leonard Hamey (lgh) at Carnegie-Mellon University
 *	Added generic_pointer and pmat.
 *
 *	Copyright (c) 1987 by Leonard Hamey
 *
 * 21-Jan-86  Leonard Hamey (lgh) at Carnegie-Mellon University
 * 	Added ucmat.
 * 25-Nov-80  David Smith (drs) at Carnegie-Mellon University
 * Changed virtual base address name to "el" for all data
 * types (Previously vali, vald, ...)  This was possible due to the
 * compiler enhancement which keeps different structure declarations
 * separate.
 *
 * 30-Oct-80  David Smith (drs) at Carnegie-Mellon University
 *	Rewritten for record-style matrices
 *
 * Aout 05 Ignacio Herrera at LAAS FR
 *      Modified for including functions
 *
 */

# ifndef matrix_h
# define matrix_h

#include "matrixStruct.h"
//#include <stdlib.h>


#ifndef INF
#define INF           3.402823466e+37
#endif

#ifndef SIGN
#define SIGN(a)       (((a) < 0 ) ? (-1) : ((a) > 0) ? (1) : (0) )
#endif

#ifndef MAX
#define MAX(a,b)      ( (a) > (b) ? (a) : (b) )
#endif

#ifndef MIN
#define MIN(a,b)      ( (a) < (b) ? (a) : (b) )
#endif

#ifndef RAD2DEG
#define RAD2DEG(a)    ((a)*180.0/(double)PI)
#endif

#ifndef DEG2RAD
#define DEG2RAD(a)    ((a)*(double)PI/180.0)
#endif

#ifndef ABS
#define ABS(a)        (((a) < 0) ? (-(a)) : (a))
#endif

#ifndef TRUNC
#define TRUNC(a)    (double)( (int)( (a) * 1000000) ) / 1000000.0
#endif

#ifndef SQRT
#define SQRT(a)             (((a) < 0 ) ? (0) : (sqrt((a))))
#endif

extern dmat newdmat (int rs, int re, int cs, int ce, int *error);
extern int matmul (dmat a, dmat b, dmat c);
extern int matSum (dmat a, dmat b, dmat c);
extern int matDif (dmat a, dmat b, dmat c);
extern int matcopy (dmat A, dmat RSLT);
extern int transpose (dmat A, dmat ATrans);
extern double matinvert (dmat a);
extern int matPrintf(dmat A);
extern int matReset(dmat A);
extern int mulEscMat(double c, dmat A, dmat B);
extern int copyReducedMat(dmat MatO, int rowsX, int colsX, dmat MatD);
extern int copyMatL2MatS(dmat MatO, dmat MatD);
extern int findZeros(dmat Vect, int *NZeros);
extern int normVect(dmat VectOr, dmat VectNor);
extern int maxVect(dmat VectOr, double *Max);
extern int minVect(dmat VectOr, double *Min);
extern int signVect(dmat VectOr, dmat VectSign);
extern int absVect(dmat VectOr, dmat VectAbs);
extern int matPrint2File(dmat A, const char *file);
extern int fillVC2Mat(dmat Vector_C, dmat Mat_A, int n);
extern int fillVR2Mat(dmat Vector_R, dmat Mat_A, int n);
extern int conca_nAB (int n, dmat VA, dmat VB, dmat VC);
extern int matSizeFile(const char *file, int *Row);
extern int File2Mat(const char *file, dmat A);
extern int findNORepeatedRows(dmat Mat, double TOL, int *NORR);
extern int matCopyNRR (dmat A, dmat RSLT, int NORR, double TOL);

extern int dmatXarmNew(dmat* dmat, int nl, int nc);


#define freemat(m) free((m).mat_sto)

# endif
