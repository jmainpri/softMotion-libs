#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include "Calcul_cg.h"
#include "Calcul_cg_modif.h"


/*double l1564(double Xf,double V0,double Tjpa,double Jmax,double Vsmp,double Vf,double Tj,double Af,double Tjndc){
  double cg;
cg = (-Xf + V0 * Tjpa + A0 * Tjpa * Tjpa / 0.2e1 + Jmax * pow(Tjpa, 0.3e1) / 0.6e1 +  Vsmp * ( (2 * Vf) -  (2 * Tj * Af) + 0.2e1 * Jmax *  (Tj * Tj) -  (2 * Vsmp) -  (2 * Amax * Tjndc) -  (2 * Amax * Tj) + Jmax *  (Tjndc * Tjndc) + 0.2e1 * Jmax *  Tjndc *  Tj) /  Amax / 0.2e1 + 0.1e1 /  Amax * pow( (2 * Vf) -  (2 * Tj * Af) + 0.2e1 * Jmax *  (Tj * Tj) -  (2 * Vsmp) -  (2 * Amax * Tjndc) -  (2 * Amax * Tj) + Jmax *  (Tjndc * Tjndc) + 0.2e1 * Jmax *  Tjndc *  Tj, 0.2e1) / 0.8e1 + ( Vf -  (Tj * Af) + Jmax *  (Tj * Tj) -  (Amax * Tjndc) -  (Amax * Tj) + Jmax *  (Tjndc * Tjndc) / 0.2e1 + Jmax *  Tjndc *  Tj) *  (Tjndc + Tj) +  (Amax *  pow( (Tjndc + Tj),  2)) / 0.2e1 - Jmax *   pow( (Tjndc + Tj),  3) / 0.6e1 + ( Vf -  (Tj * Af) + Jmax *  (Tj * Tj) -  (Amax * Tjndc) -  (Amax * Tj) + Jmax *  (Tjndc * Tjndc) / 0.2e1 + Jmax *  Tjndc *  Tj +  (Amax * (Tjndc + Tj)) - Jmax *   pow( (Tjndc + Tj),  2) / 0.2e1) *  Tj + ( Amax - Jmax *  (Tjndc + Tj)) *  (Tj * Tj) / 0.2e1 + Jmax *   pow( Tj,  3) / 0.6e1) / (0.2e1 * Jmax *  Tjndc *  Tj -  (2 * Tj * Af) +  (2 * Vf) + 0.2e1 * Jmax *  (Tj * Tj) -  (2 * Amax * Tjndc) -  (2 * Amax * Tj) + Jmax *  (Tjndc * Tjndc) +  (2 * Amax * (Tjndc + Tj)) + 0.1e1 /  Amax * ( Vf -  (Tj * Af) + Jmax *  (Tj * Tj) -  Vsmp -  (Amax * Tjndc) -  (Amax * Tj) + Jmax *  (Tjndc * Tjndc) / 0.2e1 + Jmax *  Tjndc *  Tj) * (- Af + 0.2e1 * Jmax *  Tj -  Amax + Jmax *  Tjndc) + 0.2e1 * ( Amax / 0.2e1 - Jmax *  (Tjndc + Tj) / 0.2e1) *  Tj - Jmax *   pow( (Tjndc + Tj),  2) +  Vsmp * (- Af + 0.2e1 * Jmax *  Tj -  Amax + Jmax *  Tjndc) /  Amax + (- Af + 0.2e1 * Jmax *  Tj -  Amax + Jmax *  Tjndc) *  (Tjndc + Tj) + (- Af + 0.2e1 * Jmax *  Tj + Jmax *  Tjndc - Jmax *  (Tjndc + Tj)) *  Tj);


  return cg;
}

double l1564prim(double Xf,double V0,double Tjpa,double Jmax,double Vsmp,double Vf,double Tj,double Af,double Tjndc){
  double cg;
  double a;
a=Vf - 0.1e1 / Jmax * (Amax * Amax - Af * Af) / 0.2e1 - V1;

cg = (-Xf + X1 + V1 * (a) / Amax + 0.1e1 / Amax * a*a / 0.2e1 + (Vf - 0.1e1 / Jmax * (Amax * Amax - Af * Af) / 0.2e1) * Tjn1 + Amax * Tjn1 * Tjn1 / 0.2e1 - Jmax * Tjn1*Tjn1*Tjn1 / 0.6e1 + (-Jmax * Tjn1 * Tjn1 / 0.2e1 - 0.1e1 / Jmax * Amax * Amax / 0.2e1 - 0.2e1 * Af * Tjn1 - 0.1e1 / Amax * Af*Af*Af / Jmax - 0.2e1 / Amax * Vf * Af + Amax * Af / Jmax + 0.2e1 * Vf + 0.1e1 / Jmax * Af * Af / 0.2e1 + Amax * Tjn1) * Tjpb + (0.1e1 / Amax * Vf * Jmax + 0.5e1 / 0.2e1 / Amax * Af * Af + Jmax * Tjn1 / 0.2e1 - 0.5e1 / 0.2e1 * Af) * Tjpb * Tjpb + (Jmax - 0.2e1 / Amax * Af * Jmax) * Tjpb*Tjpb*Tjpb + 0.1e1 / Amax * Jmax * Jmax * Tjpb*Tjpb*Tjpb*Tjpb / 0.2e1) / (-Jmax * Tjn1 * Tjn1 / 0.2e1 - 0.1e1 / Jmax * Amax * Amax / 0.2e1 - 0.2e1 * Af * Tjn1 - 0.1e1 / Amax * Af*Af*Af / Jmax - 0.2e1 / Amax * Vf * Af + Amax * Af / Jmax + 0.2e1 * Vf + 0.1e1 / Jmax * Af * Af / 0.2e1 + Amax * Tjn1 + (0.2e1 / Amax * Vf * Jmax + 0.5e1 / Amax * Af * Af + Jmax * Tjn1 - 0.5e1 * Af) * Tjpb + (0.3e1 * Jmax - 0.6e1 / Amax * Af * Jmax) * Tjpb * Tjpb + 0.2e1 / Amax * Jmax * Jmax * Tjpb*Tjpb*Tjpb);
  return cg;
}*/





int main(int argc, char * argv[])
{

  double a,b,c,d,e,f,g,h,i,j,k,l,m;

  double diff1 = 0;
  double diff2 = 0;

  int time_debut;
  int time_fin;
  int time;
  int n;


  a=rand()/(double)RAND_MAX;
  b=rand()/(double)RAND_MAX;
  c=rand()/(double)RAND_MAX;
  d=rand()/(double)RAND_MAX;
  e=rand()/(double)RAND_MAX;
  f=rand()/(double)RAND_MAX;
  g=rand()/(double)RAND_MAX;
  h=rand()/(double)RAND_MAX;
  i=rand()/(double)RAND_MAX;
  j=rand()/(double)RAND_MAX;
  k=rand()/(double)RAND_MAX;
  l=rand()/(double)RAND_MAX;
  m=rand()/(double)RAND_MAX;


//1357
  printf("\n1357 Base : %f, Modif : %f", l1357(a,b,c,d,e,f,g),l1357prim(a,b,c,d,e,f,g)); 
  time_debut=clock();
  for(n=0;n<100000;n++){diff1+=l1357(a,b,c,d,e,f,g);}  
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  time_debut=clock();
  for(n=0;n<100000;n++){diff2+=l1357prim(a,b,c,d,e,f,g);}
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  printf("\n diff total : %f", diff1-diff2);

//1432
  printf("\n1432 Base : %f, Modif : %f", l1432(a,b,c,d,e,f,g,h,i),l1432prim(a,b,c,d,e,f,g,h,i)); 
  time_debut=clock();
  for(n=0;n<100000;n++){diff1=l1432(a,b,c,d,e,f,g,h,i);}  
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  time_debut=clock();
  for(n=0;n<100000;n++){diff2=l1432prim(a,b,c,d,e,f,g,h,i);}
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  printf("\n diff total : %f", diff1-diff2);

//1564
/*  printf("\n1564 Base : %f, Modif : %f", l1564(a,b,c,d,e,f,g,h,i),l1564prim(a,b,c,d,e,f,g,h,i)); 
  time_debut=clock();
  for(n=0;n<100000;n++){diff1=l1564(a,b,c,d,e,f,g,h,i);}  
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  time_debut=clock();
  for(n=0;n<100000;n++){diff2=l1564prim(a,b,c,d,e,f,g,h,i);}
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  printf("\n diff total : %f", diff1-diff2);*/

//1860 
  printf("\n1860 Base : %f, Modif : %f", l1860(a,b,c,d,e,f,g,h,i,j),l1860prim(a,b,c,d,e,f,g,h,i,j)); 
  time_debut=clock();
  for(n=0;n<100000;n++){diff1=l1860(a,b,c,d,e,f,g,h,i,j);}  
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  time_debut=clock();
  for(n=0;n<100000;n++){diff2= l1860prim(a,b,c,d,e,f,g,h,i,j);}
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  printf("\n diff total : %f", diff1-diff2);

//1944
  
  printf("\n1944 Base : %f, Modif : %f", l1944(a,b,c,d,e,f,g,h,i,j,k,l,m),l1944prim(a,b,c,d,e,f,g,h,i,j,k,l,m)); 
  time_debut=clock();
  for(n=0;n<100000;n++){diff1=l1944(a,b,c,d,e,f,g,h,i,j,k,l,m);}  
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  time_debut=clock();
  for(n=0;n<100000;n++){diff2= l1944prim(a,b,c,d,e,f,g,h,i,j,k,l,m);}
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  printf("\n diff total : %f", diff1-diff2);

//2258
  printf("\n2258 Base : %f, Modif : %f", l2258(a,b,c,d,e,f,g,h,i,j,k,l),l2258prim(a,b,c,d,e,f,g,h,i,j,k,l)); 
  time_debut=clock();
  for(n=0;n<100000;n++){diff1=l2258(a,b,c,d,e,f,g,h,i,j,k,l);}  
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  time_debut=clock();
  for(n=0;n<100000;n++){diff2= l2258prim(a,b,c,d,e,f,g,h,i,j,k,l);}
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  printf("\n diff total : %f", diff1-diff2);

//2340
  printf("\n2340 Base : %f, Modif : %f", l2340(a,b,c,d,e,f,g,h,i,j),l2340prim(a,b,c,d,e,f,g,h,i,j)); 
  time_debut=clock();
  for(n=0;n<100000;n++){diff1=l2340(a,b,c,d,e,f,g,h,i,j);}  
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  time_debut=clock();
  for(n=0;n<100000;n++){diff2= l2340prim(a,b,c,d,e,f,g,h,i,j);}
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  printf("\n diff total : %f", diff1-diff2);

//2448
  printf("\n2448 Base : %f, Modif : %f", l2448(a,b,c,d,e,f,g,h,i,j),l2448prim(a,b,c,d,e,f,g,h,i,j)); 
  time_debut=clock();
  for(n=0;n<100000;n++){diff1=l2448(a,b,c,d,e,f,g,h,i,j);}  
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  time_debut=clock();
  for(n=0;n<100000;n++){diff2= l2448prim(a,b,c,d,e,f,g,h,i,j);}
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  printf("\n diff total : %f", diff1-diff2);

//2491
  printf("\n2491 Base : %f, Modif : %f", l2491(a,b,c,d,e,f,g,h,i,j,k),l2491prim(a,b,c,d,e,f,g,h,i,j,k)); 
  time_debut=clock();
  for(n=0;n<100000;n++){diff1=l2491(a,b,c,d,e,f,g,h,i,j,k);}  
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  time_debut=clock();
  for(n=0;n<100000;n++){diff2= l2491prim(a,b,c,d,e,f,g,h,i,j,k);}
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  printf("\n diff total : %f", diff1-diff2);

//2535
  printf("\n2535 Base : %f, Modif : %f", l2535(a,b,c,d,e,f,g,h,i,j,k),l2535prim(a,b,c,d,e,f,g,h,i,j,k)); 
  time_debut=clock();
  for(n=0;n<100000;n++){diff1=l2535(a,b,c,d,e,f,g,h,i,j,k);}  
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  time_debut=clock();
  for(n=0;n<100000;n++){diff2= l2535prim(a,b,c,d,e,f,g,h,i,j,k);}
  time_fin=clock();
  time=time_fin-time_debut;
  printf("\n temps d'execution moyen : %d, %f", time, ((float)time/(float)CLOCKS_PER_SEC));
  printf("\n diff total : %f\n", diff1-diff2);

  return 0;
}
