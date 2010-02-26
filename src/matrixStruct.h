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
 */

# ifndef matrix_struct_h
# define matrix_struct_h

typedef struct dmat {
  int	lb1, ub1, lb2, ub2;
  char	*mat_sto;
  double **el;
} dmat;



# endif

