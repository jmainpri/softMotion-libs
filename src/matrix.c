
/* matrix.c -- library routines for constructing dynamic matrices
 * with arbitrary bounds using Iliffe vectors
 ****************************************************************
 * HISTORY
 * 25-Nov-80  David Smith (drs) at Carnegie-Mellon University
 * Changed virtual base address name to "el" for all data
 * types (Previously vali, vald, ...)  This was possible due to the
 * compiler enhancement which keeps different structure declarations
 * separate.
 *
 * 30-Oct-80  David Smith (drs) at Carnegie-Mellon University
 *	Rewritten for record-style matrices
 *
 * 28-Oct-80  David Smith (drs) at Carnegie-Mellon University
 *	Written.
 *
 * Aout 05 Ignacio Herrera at LAAS FR
 *      Modified for including functions
 */



#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <math.h>
#include "matrixStruct.h"
#include "matrix.h"

/* typedef enum bool { */
/*   FALSE,  */
/*   TRUE, */
/* } bool; */
#define FALSE 0
#define TRUE 1

//char     *malloc ();

dmat      newdmat (int rs,int re,int cs,int ce,int* error)
{
    register double *p,
            **b;
    register int r,
              rows,
              cols;
    dmat      matrix;

    rows = re - rs + 1;
    cols = ce - cs + 1;
    if (rows <= 0 || cols <= 0) {
		*error = 1;
		return matrix;
    }

    matrix.lb1 = rs;
    matrix.ub1 = re;
    matrix.lb2 = cs;
    matrix.ub2 = ce;
    b = (double **) malloc (rows * sizeof (double *) + rows * cols * sizeof (double));
    if (b == 0) {
	*error = 1;
	return (matrix);
    }
    matrix.mat_sto = (char *) b;
    *error = 0;
    p = ((double *) &b[rows]) - cs;
    matrix.el = b -= rs;
    for (r = rs; r <= re; r++) {
	b[r] = p;
	p += cols;
    }

    return matrix;
}

int matmul (dmat a, dmat b, dmat c)
{
    register int i,
              j,
              k;
    register int broff,
              croff,
              ccoff;		/* coordinate origin offsets */
    register double t;
    dmat      d;		/* Temp. workspace matrix */
    int      tempflag;
    int       error;

    if (a.ub2 - a.lb2 != b.ub1 - b.lb1
	|| a.ub1 - a.lb1 != c.ub1 - c.lb1
	|| b.ub2 - b.lb2 != c.ub2 - c.lb2
     ) {
	errno = EDOM;
	return (-1);
    }
    if (a.mat_sto != c.mat_sto && b.mat_sto != c.mat_sto) {
	d = c;
	tempflag = FALSE;
    } else {
	d = newdmat (c.lb1, c.ub1, c.lb2, c.ub2, &error);
	if (error) {
	    fprintf (stderr, "Matmul: out of storage.\n");
	    errno = ENOMEM;
	    return (-1);
	}
	tempflag = TRUE;
    }
    broff = b.lb1 - a.lb2;	/* B's row offset from A */
    croff = c.lb1 - a.lb1;	/* C's row offset from A */
    ccoff = c.lb2 - b.lb2;	/* C's column offset from B */
    for (i = a.lb1; i <= a.ub1; i++)
	for (j = b.lb2; j <= b.ub2; j++) {
	    t = 0.0;
	    for (k = a.lb2; k <= a.ub2; k++)
		t += a.el[i][k] * b.el[k + broff][j];
	    d.el[i + croff][j + ccoff] = t;
	}

    if (tempflag) {
	for (i = c.lb1; i <= c.ub1; i++)
	    for (j = c.lb2; j <= c.ub2; j++)
		c.el[i][j] = d.el[i][j];
	freemat (d);
    }
    return (0);
}

int matSum (dmat a,dmat  b,dmat  c)
{
  register int i,j;

  if (a.ub1 - a.lb1 != b.ub1 - b.lb1 || a.ub2 - a.lb2 != b.ub2 - b.lb2 || b.ub1 - b.lb1 != c.ub1 - c.lb1 || b.ub2 - b.lb2 != c.ub2 - c.lb2 ) {
    fprintf (stderr, "matSum: Different Size Matrix\n");
    errno = EDOM;
    return -1;
  }
  for (i = a.lb1; i <= a.ub1; i++)
    for (j = a.lb2; j <= a.ub2; j++)
	 c.el[i][j] =  a.el[i][j] + b.el[i][j];
  return (0);
}

int matDif (dmat a, dmat b,dmat  c)
{
  register int i,j;

  if (a.ub1 - a.lb1 != b.ub1 - b.lb1 || a.ub2 - a.lb2 != b.ub2 - b.lb2 || b.ub1 - b.lb1 != c.ub1 - c.lb1 || b.ub2 - b.lb2 != c.ub2 - c.lb2 ) {
    fprintf (stderr, "matSum: Different Size Matrix\n");
    errno = EDOM;
    return -1;
  }
  for (i = a.lb1; i <= a.ub1; i++)
    for (j = a.lb2; j <= a.ub2; j++)
	 c.el[i][j] =  a.el[i][j] - b.el[i][j];
  return (0);
}

int matcopy ( dmat  A,  dmat  RSLT)
{
    register int i,
              j,
              rowsize,
              colsize;
    register double **a = A.el,
            **rslt = RSLT.el;

    rowsize = A.ub1 - A.lb1;
    colsize = A.ub2 - A.lb2;
    if (rowsize != RSLT.ub1 - RSLT.lb1 || colsize != RSLT.ub2 - RSLT.lb2)
	return (-1);

    for (i = 0; i <= rowsize; i++)
	for (j = 0; j <= colsize; j++)
	    rslt[RSLT.lb1 + i][RSLT.lb2 + j] = a[A.lb1 + i][A.lb2 + j];

    return (0);
}

int transpose (dmat A, dmat ATrans)
{
    register int i,
              j,
              rowsize,
              colsize;
    register double **a = A.el,
            **atrans = ATrans.el;
    int       error;
    double    temp;
    dmat      TMP;

    rowsize = A.ub1 - A.lb1;
    colsize = A.ub2 - A.lb2;
    if (rowsize < 0 || rowsize != ATrans.ub2 - ATrans.lb2 ||
	colsize < 0 || colsize != ATrans.ub1 - ATrans.lb1)
	return (-1);

    if (A.mat_sto == ATrans.mat_sto
	&& A.lb1 == ATrans.lb1
	&& A.lb2 == ATrans.lb2) {
	for (i = 0; i <= rowsize; i++)
	    for (j = i + 1; j <= colsize; j++) {
		temp = a[A.lb1 + i][A.lb2 + j];
		atrans[A.lb1 + i][A.lb2 + j] = a[A.lb1 + j][A.lb2 + i];
		atrans[A.lb1 + j][A.lb2 + i] = temp;
	    }
    } else if (A.mat_sto == ATrans.mat_sto) {
	TMP = newdmat (ATrans.lb1, ATrans.ub1, ATrans.lb2, ATrans.ub2, &error);
	if (error)
	    return (-2);

	for (i = 0; i <= rowsize; i++)
	    for (j = 0; j <= colsize; j++) {
		TMP.el[ATrans.lb1 + j][ATrans.lb2 + i] =
		 a[A.lb1 + i][A.lb2 + j];
	    }

	matcopy (TMP, ATrans);
	freemat (TMP);
    } else {
	for (i = 0; i <= rowsize; i++)
	    for (j = 0; j <= colsize; j++)
		atrans[ATrans.lb1 + j][ATrans.lb2 + i]
		 = a[A.lb1 + i][A.lb2 + j];
    }

    return (0);
}


/* Matinvert.c -- In-place matrix inversion using full pivoting.
 * The standard Gauss-Jordan method is used.  The return value
 * is the determinant.
 * Call:  det = matinvert(a);
 ****************************************************************
 * HISTORY
 * 26-Feb-82  David Smith (drs) at Carnegie-Mellon University
 *	Written.
 *
 */

#define PERMBUFSIZE 100		/* Mat bigger than this requires calling malloc. */

#define abs(x) ((x)>=0 ? (x) : -(x))

double    matinvert (dmat a)
{
    int register  i, j, k;
    double    det, biga, hold;
    int      *l, *m;		/* Row and column permutation vectors */
    int       permbuf[2 * PERMBUFSIZE];
    int       mallocflag;

    if (a.lb1 != a.lb2 || a.ub1 != a.ub2)
	return (0.0);

    /* Allocate permutation vectors for l and m, with the same origin as the matrix. */
    if (a.ub1 - a.lb1 + 1 <= PERMBUFSIZE) {
	l = permbuf;
	mallocflag = 0;
    } else {
	l = (int *) malloc (2 * (a.ub1 - a.lb1 + 1) * sizeof (int));
	if (l == 0) {
	    fprintf (stderr, "matinvert: can't get working storage.\n");
	    errno = ENOMEM;
	    return 0.0;
	}
	mallocflag = 1;
    }
    l -= a.lb1;
    m = l + (a.ub1 - a.lb1 + 1);

    det = 1.0;
    for (k = a.lb1; k <= a.ub1; k++) {
	l[k] = k;
	m[k] = k;
	biga = a.el[k][k];

	/* Find the biggest element in the submatrix */
	for (i = k; i <= a.ub1; i++)
	    for (j = k; j <= a.ub2; j++)
		if (abs (a.el[i][j]) > abs (biga)) {
		    biga = a.el[i][j];
		    l[k] = i;
		    m[k] = j;
		}
	/* Interchange rows */
	i = l[k];
	if (i > k)
	    for (j = a.lb2; j <= a.ub2; j++) {
		hold = -a.el[k][j];
		a.el[k][j] = a.el[i][j];
		a.el[i][j] = hold;
	    }

	/* Interchange columns */
	j = m[k];
	if (j > k)
	    for (i = a.lb1; i <= a.ub1; i++) {
		hold = -a.el[i][k];
		a.el[i][k] = a.el[i][j];
		a.el[i][j] = hold;
	    }

	/* Divide column by minus pivot (value of pivot element is contained in biga). */
	if (biga == 0.0)
	    return (0.0);
	for (i = a.lb1; i <= a.ub1; i++)
	    if (i != k)
		a.el[i][k] /= -biga;

	/* Reduce matrix */
	for (i = a.lb1; i <= a.ub1; i++)
	    if (i != k) {
		hold = a.el[i][k];
		for (j = a.lb2; j <= a.ub2; j++)
		    if (j != k)
			a.el[i][j] += hold * a.el[k][j];
	    }
	/* Divide row by pivot */
	for (j = a.lb2; j <= a.ub2; j++)
	    if (j != k)
		a.el[k][j] /= biga;

	det *= biga;		/* Product of pivots */
	a.el[k][k] = 1.0 / biga;
    }				/* K loop */

    /* Final row & column interchanges */
    for (k = a.ub1 - 1; k >= a.lb1; k--) {
	i = l[k];
	if (i > k)
	    for (j = a.lb2; j <= a.ub2; j++) {
		hold = a.el[j][k];
		a.el[j][k] = -a.el[j][i];
		a.el[j][i] = hold;
	    }
	j = m[k];
	if (j > k)
	    for (i = a.lb1; i <= a.ub1; i++) {
		hold = a.el[k][i];
		a.el[k][i] = -a.el[j][i];
		a.el[j][i] = hold;
	    }
    }

    if (mallocflag)
	free (l + a.lb1);
    return det;
}

int matPrintf(dmat A)
{
  register int i,j;
  int rowsize, colsize;

   rowsize = A.ub1 - A.lb1;
   colsize = A.ub2 - A.lb2;

   for (i = 0; i <= rowsize; i++) {
	for (j = 0; j <= colsize; j++)
	  printf("[ %f ] \t",A.el[i][j]);
     printf("\n");
   }
   return (0);
}

int matReset(dmat A)
{
  register int i,j;
  int rowsize, colsize;

   rowsize = A.ub1 - A.lb1;
   colsize = A.ub2 - A.lb2;

   for (i = 0; i <= rowsize; i++)
	for (j = 0; j <= colsize; j++)
	  A.el[i][j] = 0;

   return (0);
}

int mulEscMat(double c,dmat A, dmat B)
{
  register int i,j;
  int rowsize, colsize;
  register double **a = A.el,
                  **b = B.el;

  rowsize = A.ub1 - A.lb1;
  colsize = A.ub2 - A.lb2;

  if (rowsize != B.ub1 - B.lb1 || colsize != B.ub2 - B.lb2)
    return (-1);

  for (i = 0; i <= rowsize; i++)
    for (j = 0; j <= colsize; j++)
	 b[B.lb1 + i][B.lb2 + j] = c*a[A.lb1 + i][A.lb2 + j];

  return (0);
}


/* Copy reduced matrix
   ignoring the first rowsX rows and the first colsX columns
*/
int copyReducedMat(dmat MatO,int rowsX,int colsX,dmat MatD)
{
  register int rowsize, colsize;
  register int i, j, ia=0, ja=0;

  rowsize = MatO.ub1 - MatO.lb1;
  colsize = MatO.ub2 - MatO.lb2;

  if (rowsize-rowsX != MatD.ub1-MatD.lb1 || colsize-colsX != MatD.ub2-MatD.lb2) {
    fprintf (stderr, "Matrix Dimension Error in copyReducedMat\n");
    return (-1);
  }

  for (i = rowsX; i <= rowsize; i++) {
    for (j = colsX; j <= colsize; j++) {
	 MatD.el[MatD.lb1 + ia][MatD.lb2 + ja] = MatO.el[MatO.lb1 + i][MatO.lb2 + j];
	 ja ++;
    }
    ia ++;
  }

  return (0);
}
/* Copy the first rows and columns of MatO to MatD */
int copyMatL2MatS(dmat MatO,dmat MatD)
{
  register int rowsize, colsize;
  register int i, j, ia=0, ja=0;

  rowsize = MatD.ub1 - MatD.lb1;
  colsize = MatD.ub2 - MatD.lb2;

  if (rowsize > MatO.ub1-MatO.lb1 || colsize > MatO.ub2-MatO.lb2) {
    fprintf (stderr, "Matrix Dimension Error in copyMatL2MatS\n");
    return (-1);
  }

  for (i = MatD.lb1; i <= MatD.ub1; i++) {
    for (j = MatD.lb2; j <= MatD.ub2; j++) {
	 MatD.el[i][j] = MatO.el[MatO.lb1 + ia][MatO.lb2 + ja];
	 ja++;
    }
    ia++;
  }

  return (0);
}

/* Find how many first Zero elements are in a Vector */
int findZeros(dmat Vect,int *NZeros)
{
  register int rowsize, colsize;
  register int i, j;
  register int flag =1;

  rowsize = Vect.ub1 - Vect.lb1;
  colsize = Vect.ub2 - Vect.lb2;

  if ((rowsize!=0 && colsize!= 0)||(rowsize==0 && colsize== 0)) {
    fprintf (stderr, "Error - Vector Dimensions \n");
    return (-1);
  }

  if (rowsize==0)
    for (j = 0; j <= colsize; j++)
     if (Vect.el[0][j]!=0 && flag==1 ) {
	 *NZeros = j;
	 flag = 0;
     }

  if (colsize==0)
    for (i = 0; i <= rowsize; i++)
     if (Vect.el[i][0]!=0 && flag==1 ) {
	 *NZeros = i;
	 flag = 0;
     }
  return (0);
}

/* Normalize Vector */
/* Gets Unitary Vector Norm */
int normVect(dmat VectOr,dmat VectNor)
{
  register int rowsize, colsize;
  register int i, j;
  double norm = 0.0;

  rowsize = VectOr.ub1 - VectOr.lb1;
  colsize = VectOr.ub2 - VectOr.lb2;

  if (rowsize != VectNor.ub1 - VectNor.lb1 || colsize != VectNor.ub2 - VectNor.lb2) {
    fprintf (stderr, "Error - Vector Dimensions Original - Normalized \n");
    return (-1);
  }
  if ((rowsize!=0 && colsize!= 0)||(rowsize==0 && colsize== 0)) {
    fprintf (stderr, "Error - Vector Dimensions \n");
    return (-1);
  }
  /* For Row vector */
  if (rowsize==0) {
    for (j = 0; j <= colsize; j++)
	 norm += VectOr.el[0][j]*VectOr.el[0][j];
    if (norm == 0.0) {
	 fprintf(stderr,"Error - Vector Zero \n");
	 return (-1);
    }
    for (j = 0; j <= colsize; j++)
	 VectNor.el[0][j] = VectOr.el[0][j]/norm;
  }
  /* For Column vector */
  if (colsize==0) {
    for (i = 0; i <= rowsize; i++)
	 norm += VectOr.el[i][0]*VectOr.el[i][0];
    if (norm == 0.0) {
	 fprintf(stderr,"Error - Vector Zero \n");
	 return (-1);
    }
    norm = sqrt(norm);
    for (i = 0; i <= rowsize; i++)
	 VectNor.el[i][0] = VectOr.el[i][0]/norm;
  }
  return (0);
}

/* Gets the Max Element of a vector */
int maxVect(dmat VectOr, double *Max)
{
  register int rowsize, colsize;
  register int i;

  rowsize = VectOr.ub1 - VectOr.lb1;
  colsize = VectOr.ub2 - VectOr.lb2;

  if ((rowsize != 0 && colsize != 0) ||(rowsize == 0 && colsize == 0)) {
    fprintf (stderr, "Error - Vector Dimensions - Max Element \n");
    return (-1);
  }
  /* Max Initialization */
  *Max= -INF;
  /* For ROW Vector */
  if (rowsize==0)
    for (i=0; i<= colsize; i++)
	 *Max = MAX(VectOr.el[0][i],*Max);

  /* For COL Vector */
  if (colsize==0)
    for (i=0; i<= rowsize; i++)
	 *Max = MAX(VectOr.el[i][0],*Max);

  return (0);
}
/* Gets the Min Element of a vector */
int minVect(dmat VectOr, double *Min)
{
  register int rowsize, colsize;
  register int i;

  rowsize = VectOr.ub1 - VectOr.lb1;
  colsize = VectOr.ub2 - VectOr.lb2;

  if ((rowsize != 0 && colsize != 0) ||(rowsize == 0 && colsize == 0)) {
    fprintf (stderr, "Error - Vector Dimensions - Max Element \n");
    return (-1);
  }
  /* Max Initialization */
  *Min= INF;
  /* For ROW Vector */
  if (rowsize==0)
    for (i=0; i<= colsize; i++)
	 *Min = MIN(VectOr.el[0][i],*Min);

  /* For COL Vector */
  if (colsize==0)
    for (i=0; i<= rowsize; i++)
	 *Min = MIN(VectOr.el[i][0],*Min);

  return (0);
}

/* Gets a vector with the sign of every elementor zero */
int signVect(dmat VectOr, dmat VectSign)
{
  register int rowsize, colsize;
  register int i;

  rowsize = VectOr.ub1 - VectOr.lb1;
  colsize = VectOr.ub2 - VectOr.lb2;

  if (rowsize != VectSign.ub1 - VectSign.lb1 || colsize != VectSign.ub2 - VectSign.lb2) {
    fprintf (stderr, "Error - Vector Dimensions Original - Sign Vector \n");
    return (-1);
  }

  if ((rowsize!=0 && colsize!= 0)||(rowsize==0 && colsize== 0)) {
    fprintf (stderr, "Error - Vector Dimensions - Sign Vector\n");
    return (-1);
  }

  /* For Row vector */
  if (rowsize==0) {
    for (i = 0; i <= colsize; i++)
	 if (VectOr.el[0][i]!=0.0)
	   VectSign.el[0][i]=SIGN(VectOr.el[0][i]);
	 else
	   VectSign.el[0][i]=0;
  }
  /* For Column vector */
  if (colsize==0) {
    for (i = 0; i <= rowsize; i++)
	 if (VectOr.el[i][0]!=0.0)
	   VectSign.el[i][0]=SIGN(VectOr.el[i][0]);
	 else
	   VectSign.el[i][0]=0;
  }
  return (0);
}

/* Gets a vector with the sign ABS of every element */
int absVect(dmat VectOr, dmat VectAbs)
{
  register int rowsize, colsize;
  register int i;

  rowsize = VectOr.ub1 - VectOr.lb1;
  colsize = VectOr.ub2 - VectOr.lb2;

  if (rowsize != VectAbs.ub1 - VectAbs.lb1 || colsize != VectAbs.ub2 - VectAbs.lb2) {
    fprintf (stderr, "Error - Vector Dimensions Original - Abs Vector \n");
    return (-1);
  }

  if ((rowsize!=0 && colsize!= 0)||(rowsize==0 && colsize== 0)) {
    fprintf (stderr, "Error - Vector Dimensions - Abs Vector \n");
    return (-1);
  }

  /* For Row vector */
  if (rowsize==0)
    for (i = 0; i <= colsize; i++)
	 VectAbs.el[0][i]=ABS(VectOr.el[0][i]);

  /* For Column vector */
  if (colsize==0)
    for (i = 0; i <= rowsize; i++)
	 VectAbs.el[i][0]=ABS(VectOr.el[i][0]);

  return (0);
}

/* Print a Matrix in a File (file) */
int matPrint2File(dmat A, const char *file)
{
  FILE *fileptr;
  register int rowsize, colsize;
  register int i,j;

  if ((fileptr = fopen(file,"wb"))==NULL) {
    fprintf(stderr,"matFilePrintf - ERROR - Open File \n");
    return (-1);
  }

  rowsize = A.ub1 - A.lb1;
  colsize = A.ub2 - A.lb2;

  if ((rowsize == 0 && colsize == 0)) {
    fprintf (stderr, "matFilePrintf - ERROR - Matrix Dimensions \n");
    return (-1);
  }

  for (i = 0; i <= rowsize; i++) {
    for (j = 0; j <= colsize; j++)
	 fprintf(fileptr,"%f ",A.el[i][j]);
    fprintf(fileptr,"\n");
  }

  fclose(fileptr);
  return (0);
}

/* Fill a matrix's column (n) with a column vector */
int fillVC2Mat(dmat Vector_C,dmat Mat_A,int n)
{
  register int rowsize, colsize;
  register int i, ia=0;

  rowsize = Mat_A.ub1 - Mat_A.lb1;
  colsize = Mat_A.ub2 - Mat_A.lb2;

  if (rowsize != Vector_C.ub1-Vector_C.lb1 || colsize <= Vector_C.ub2-Vector_C.lb2) {
    fprintf (stderr, "fillVC2Mat - ERROR - Matrix & Vector Dimensions \n");
    return (-1);
  }

  for (i = Mat_A.lb1; i <= Mat_A.ub1; i++) {
    Mat_A.el[Mat_A.lb1 + ia][Mat_A.lb2 + n] = Vector_C.el[Vector_C.lb1+ia][Vector_C.lb2];
    ia ++;
  }

  return (0);
}

/* Fill a  matrix's row (n) with a row vector */
int fillVR2Mat(dmat Vector_R,dmat Mat_A,int n)
{
  register int rowsize, colsize;
  register int j, ja=0;

  rowsize = Mat_A.ub1 - Mat_A.lb1;
  colsize = Mat_A.ub2 - Mat_A.lb2;

  if (rowsize <= Vector_R.ub1-Vector_R.lb1 || colsize != Vector_R.ub2-Vector_R.lb2) {
    fprintf (stderr, "fillVR2Mat - ERROR - Matrix & Vector Dimensions \n");
    return (-1);
  }

  for (j = Mat_A.lb2; j <= Mat_A.ub2; j++) {
    Mat_A.el[Mat_A.lb1 + n][Mat_A.lb2 + ja] = Vector_R.el[Vector_R.lb1][Vector_R.lb2+ja];
    ja ++;
  }

  return (0);
}

/* Concatenate Vectors: Concatenate the n first elements of V_A with V_B
                           V_C = { V_A[n]   V_B }                             */
int conca_nAB (int n,dmat  VA, dmat VB, dmat VC)
{
  int la, lb, lc;
  register int rsA, rsB, rsC, csA, csB, csC;
  register int i;

  rsA = VA.ub1 - VA.lb1;
  csA = VA.ub2 - VA.lb2;

  rsB = VB.ub1 - VB.lb1;
  csB = VB.ub2 - VB.lb2;

  rsC = VC.ub1 - VC.lb1;
  csC = VC.ub2 - VC.lb2;

  /* ROW CASE */
  if (rsA == 0 && rsB== 0 && rsC == 0) {
    la = csA+1;
    lb = csB+1;
    lc = csC+1;
    if (n <= la && n>-1) {
	 if (lc < n + lb) {
	   fprintf(stderr, "ERROR - Vector C Dimensions \n");
	   return -1;
	 }
	 for (i=0;i<n; i++)
	   VC.el[0][i]=VA.el[0][i];

	 for (i=n;i<lb+n; i++)
	   VC.el[0][i]=VB.el[0][i-n];
	 if (lb+n < lc)
	   for (i=lb+n;i<lc;i++)
		VC.el[0][i]=VC.el[0][lb+n-1];
	 return 0;
    }
    else {
	 fprintf(stderr, " ERROR - Bad Number of Elements - ROW CASE \n");
	 return -1;
    }
  }

  /* COLUMN CASE */
  if (csA == 0 && csB== 0 && csC == 0) {
    la = rsA+1;
    lb = rsB+1;
    lc = rsC+1;
    if (n <= la && n>-1) {
	 if (lc < n + lb) {
	   fprintf(stderr, "ERROR - Vector C Dimensions \n");
	   return -1;
	 }
	 for (i=0;i<n; i++)
	   VC.el[i][0]=VA.el[i][0];

	 for (i=n;i<lb+n; i++)
	   VC.el[i][0]=VB.el[i-n][0];
	 if (lb+n < lc)
	   for (i=lb+n;i<lc;i++)
		VC.el[i][0]=VC.el[lb+n-1][0];
	 return 0;
    }
    else {
	 fprintf(stderr, " ERROR - Bad Number of Elements - COLUMN CASE \n");
	 return -1;
    }
  }

  fprintf(stderr, " ERROR - Vectors Dimensions ");
  return -1;
}

/* Print a Matrix in a File (file) */
int matSizeFile(const char *file, int *Row)
{
  FILE *fileptr;
  int rows;

  /* Open File Pointer */
  if ((fileptr = fopen(file,"r"))==NULL) {
    fprintf(stderr,"matFilePrintf - ERROR - Open File \n");
    return (-1);
  }

  rows=0;
  while(!feof(fileptr)) {
    /* Man ASCII   10 = \n */
    if  (fgetc(fileptr)==10 )
	 rows ++;
  }

  /* Close file pointer */
  fclose(fileptr);

  *Row=rows;

  return (0);
}

/*  Text File to Matrix, using Tabs for separation, 6 elements  */
int File2Mat(const char *file, dmat A)
{
  FILE *fileptr;
  register int rowsize, colsize;
  register int j;
  float a,b,c,d,e,f;

  if ((fileptr = fopen(file,"r"))==NULL) {
    fprintf(stderr,"matFilePrintf - ERROR - Open File \n");
    return (-1);
  }

  rowsize = A.ub1 - A.lb1;
  colsize = A.ub2 - A.lb2;

  if ((rowsize == 0 && colsize == 0)) {
    fprintf (stderr, "matFilePrintf - ERROR - Matrix Dimensions \n");
    return (-1);
  }

  j=0;
  while(!feof(fileptr)) {
    fscanf(fileptr, "%f\t%f\t%f\t%f\t%f\t%f\n",&a,&b,&c,&d,&e,&f);
    A.el[A.lb1+j][A.lb2+0] = a;
    A.el[A.lb1+j][A.lb2+1] = b;
    A.el[A.lb1+j][A.lb2+2] = c;
    A.el[A.lb1+j][A.lb2+3] = d;
    A.el[A.lb1+j][A.lb2+4] = e;
    A.el[A.lb1+j][A.lb2+5] = f;
    j++;
  }

  fclose(fileptr);

  return (0);
}

/* Find how many repeated rows, TOLerance */
int findNORepeatedRows(dmat Mat, double TOL,  int *NORR)
{
  register int rowsize, colsize;
  register int i, j;
  double SumQ, Dif;
  int Nor;

  rowsize = Mat.ub1 - Mat.lb1;
  colsize = Mat.ub2 - Mat.lb2;
  Nor = 0;

  for (i = Mat.lb1; i <= Mat.ub1-1; i++) {
    SumQ = 0;
    for (j = Mat.lb2; j <= Mat.ub2; j++) {
	 Dif  = Mat.el[i+1][j]-Mat.el[i][j];
	 SumQ += Dif*Dif;
    }
    SumQ = sqrt(SumQ);

    if (SumQ< TOL)
	 Nor++;
  }
  *NORR = Nor;

  return (0);
}

int matCopyNRR (dmat A,dmat  RSLT,int  NORR,double TOL)
{
  register int i, j, ia;
  int  rowsize, colsize;
  double SumQ, Dif;

  rowsize = A.ub1 - A.lb1;
  colsize = A.ub2 - A.lb2;

  if (rowsize != RSLT.ub1 - RSLT.lb1 + NORR || colsize != RSLT.ub2 - RSLT.lb2)
    return (-1);

  ia=0;
  for (i = A.lb1; i <= A.ub1; i++) {

    SumQ = 0;

    if (i<A.ub1)
	 for (j = A.lb2; j <= A.ub2; j++) {
	   Dif  = A.el[i+1][j]-A.el[i][j];
	   SumQ += Dif*Dif;
	 }
    else
	 SumQ=100*TOL;

    SumQ=sqrt(SumQ);

    if (SumQ>TOL) {
	 for (j = 0; j <= colsize; j++)
	   RSLT.el[RSLT.lb1+ia][RSLT.lb2+j]=A.el[i][A.lb2+j];
	 ia++;
    }
  }
  return (0);
}

/*
 *  dmatXarmNew : to simplify allocation of dmat in xarm
 */
int dmatXarmNew(dmat* dmat, int nl, int nc)
{
  int error;
  *dmat = newdmat(0, nl, 0, nc, &error);
  return error;
}
