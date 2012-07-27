#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "Calcul_cg_modif.h"

// le numero de fonction correspond a la ligne d'origine dans le fichier softMotion.c

double l1357prim (double Xf,double V0,double Tj,double Tj01,double A0,double Jmax,double Tj02){
  double cg;
  double a,b,c;
a=Tj-Tj01;
b=Tj+Tj02;
c=(A0+Jmax*a);
cg=(-Xf+V0*a+A0*a*a/2.0+Jmax*a*a*a/6.0+2.0*(V0+A0*a+Jmax*a*a/2.0)*Tj+2.0*c*Tj*Tj-4.0/3.0*Jmax*Tj*Tj*Tj+(V0+A0*a+Jmax*a*a/2.0)*b-c*b*b/2.0+Jmax*b*b*b/6.0)/(4.0*V0+4.0*A0*a+2.0*Jmax*a*a-2.0*Jmax*Tj*Tj+6.0*c*Tj);
  return cg;
}


double l1432prim(double Xf,double V1,double Amax,double Af,double Tjn1,double Jmax,double Tjpb,double X1,double Vf){
  double cg;
  double a;
a=Vf - 1.0 / Jmax * (Amax * Amax - Af * Af) / 2.0 - V1;

cg = (-Xf + X1 + V1 * (a) / Amax + 1.0 / Amax * a*a / 2.0 + (Vf - 1.0 / Jmax * (Amax * Amax - Af * Af) / 2.0) * Tjn1 + Amax * Tjn1 * Tjn1 / 2.0 - Jmax * Tjn1*Tjn1*Tjn1 / 6.0 + (-Jmax * Tjn1 * Tjn1 / 2.0 - 1.0 / Jmax * Amax * Amax / 2.0 - 2.0 * Af * Tjn1 - 1.0 / Amax * Af*Af*Af / Jmax - 2.0 / Amax * Vf * Af + Amax * Af / Jmax + 2.0 * Vf + 1.0 / Jmax * Af * Af / 2.0 + Amax * Tjn1) * Tjpb + (1.0 / Amax * Vf * Jmax + 5.0 / 2.0 / Amax * Af * Af + Jmax * Tjn1 / 2.0 - 5.0 / 2.0 * Af) * Tjpb * Tjpb + (Jmax - 2.0 / Amax * Af * Jmax) * Tjpb*Tjpb*Tjpb + 1.0 / Amax * Jmax * Jmax * Tjpb*Tjpb*Tjpb*Tjpb / 2.0) / (-Jmax * Tjn1 * Tjn1 / 2.0 - 1.0 / Jmax * Amax * Amax / 2.0 - 2.0 * Af * Tjn1 - 1.0 / Amax * Af*Af*Af / Jmax - 2.0 / Amax * Vf * Af + Amax * Af / Jmax + 2.0 * Vf + 1.0 / Jmax * Af * Af / 2.0 + Amax * Tjn1 + (2.0 / Amax * Vf * Jmax + 5.0 / Amax * Af * Af + Jmax * Tjn1 - 5.0 * Af) * Tjpb + (3.0 * Jmax - 6.0 / Amax * Af * Jmax) * Tjpb * Tjpb + 2.0 / Amax * Jmax * Jmax * Tjpb*Tjpb*Tjpb);
  return cg;
}

double l1860prim(double Xf,double V0,double Tj,double Tjndc,double A0,double Jmax,double Tjpb, double Af, double Vf, double Amax){
  double cg;
  double k;
  k= Tj + Tjndc;
  cg = (-Xf + V0 * Tj + A0 * Tj * Tj / 2.0 + Jmax * Tj*Tj*Tj / 6.0 + (V0 + A0 * Tj + Jmax * Tj * Tj / 2.0) * k + (A0 + Jmax * Tj) * k*k / 2.0 - Jmax * k*k*k / 6.0 + (V0 + A0 * Tj + Jmax * Tj * Tj / 2.0 + (A0 + Jmax * Tj) * k - Jmax * k*k / 2.0) * (-2.0 * Vf * Jmax + 2.0 * Jmax * Tjpb * Af - Jmax * Jmax * Tjpb * Tjpb - Amax * Amax + A0 * A0 + 4.0 * A0 * Jmax * Tj + 2.0 * Jmax * Jmax * Tj * Tj + 2.0 * V0 * Jmax) / Jmax / Amax / 2.0 - 1.0 / Amax * (-2.0 * Vf * Jmax + 2.0 * Jmax * Tjpb * Af - Jmax * Jmax * Tjpb * Tjpb - Amax * Amax + A0 * A0 + 4.0 * A0 * Jmax * Tj + 2.0 * Jmax * Jmax * Tj * Tj + 2.0 * V0 * Jmax)*(-2.0 * Vf * Jmax + 2.0 * Jmax * Tjpb * Af - Jmax * Jmax * Tjpb * Tjpb - Amax * Amax + A0 * A0 + 4.0 * A0 * Jmax * Tj + 2.0 * Jmax * Jmax * Tj * Tj + 2.0 * V0 * Jmax) * pow(Jmax, -2.0) / 8.0 + (V0 + A0 * Tj + Jmax * Tj * Tj / 2.0 + (A0 + Jmax * Tj) * k - Jmax * k*k / 2.0 - (-2.0 * Vf * Jmax + 2.0 * Jmax * Tjpb * Af - Jmax * Jmax * Tjpb * Tjpb - Amax * Amax + A0 * A0 + 4.0 * A0 * Jmax * Tj + 2.0 * Jmax * Jmax * Tj * Tj + 2.0 * V0 * Jmax) / Jmax / 2.0) * Tjpb - Amax * Tjpb * Tjpb / 2.0 + Jmax * Tjpb*Tjpb*Tjpb / 6.0) / (2.0 * V0 + 2.0 * A0 * Tj + Jmax * Tj * Tj + (A0 + Jmax * Tj) * k + 2.0 * (A0 / 2.0 + Jmax * Tj / 2.0) * k + (2.0 * A0 + 2.0 * Jmax * Tj) * (-Vf * Jmax + Jmax * Tjpb * Af - Jmax * Jmax * Tjpb * Tjpb / 2.0 - Amax * Amax / 2.0 + A0 * A0 / 2.0 + 2.0 * A0 * Jmax * Tj + Jmax * Jmax * Tj * Tj + V0 * Jmax) / Jmax / Amax + (V0 + A0 * Tj + Jmax * Tj * Tj / 2.0 + (A0 + Jmax * Tj) * (Tj + Tjndc) - Jmax * k*k / 2.0) * (2.0 * A0 * Jmax + 2.0 * Jmax * Jmax * Tj) / Jmax / Amax - 1.0 / Amax * (-Vf * Jmax + Jmax * Tjpb * Af - Jmax * Jmax * Tjpb * Tjpb / 2.0 - Amax * Amax / 2.0 + A0 * A0 / 2.0 + 2.0 * A0 * Jmax * Tj + Jmax * Jmax * Tj * Tj + V0 * Jmax) * pow(Jmax, -2.0) * (2.0 * A0 * Jmax + 2.0 * Jmax * Jmax * Tj) + (2.0 * A0 + 2.0 * Jmax * Tj - (2.0 * A0 * Jmax + 2.0 * Jmax * Jmax * Tj) / Jmax) * Tjpb);
  return cg;
}




double l1944prim(double Xf,double X0, double V0,double Tj1,double Tjndc,double A0,double Jmax,double Tjpdc, double Af, double Vf, double Amax, double A1dc, double V1dc){
  double cg;
  double a,b,b2,c,d,e,f,g;
a=Jmax * Tj1;
b=Tjpdc + Tj1;
b2=b*b / 2.0;
c=(2 * Af * Af) - 4.0 * Vf * Jmax + 4.0 * Jmax * a * Tj1 + 4.0 * V1dc * Jmax + 8.0* A1dc * a + 2.0 * A1dc * A1dc;
d=Af + sqrt( c) / 2.0;
e=Tjndc + Tj1;
f=e + ( d) / Jmax;
g=8.0* Jmax * a + 8.0* A1dc * Jmax;

cg = (-Xf + X0 + V0 * (b) + A0 * b2 + Jmax * b*b*b / 0.6e1 + (V0 + A0 * (b) + Jmax * b2) * (f) + (A0 / 2.0 + Jmax * (b) / 2.0) * f*f - Jmax * f*f*f / 6.0 + (Vf + sqrt( c) * ( d) / Jmax / 2.0 - 1.0 / Jmax * d*d / 2.0) * ( d) / Jmax - sqrt( c) * d*d * pow(Jmax, -2.0) / 4.0 + pow(Jmax, -2.0) * d*d*d / 0.6e1) / (V0 + A0 * (b) + Jmax * b2 + (A0 + Jmax * (b)) * (f) + (V0 + A0 * (b) + Jmax * b2) * (1.0 + pow( c, -1.0 / 2.0) * (g) / Jmax / 4.0) + Jmax * f*f / 2.0 + (A0 + Jmax * (b)) * (f) * (1.0 + pow( c, -1.0 / 2.0) * (g) / Jmax / 4.0) -  f*f * (Jmax + pow( c, -1.0 / 2.0) * (g) / 4.0) / 2.0 + (Vf / 4.0 + sqrt( c) * ( d) / Jmax / 8.0- 1.0 / Jmax * d*d / 8.0) * pow( c, -1.0 / 2.0) * (g) / Jmax);

return cg;
}


double l2258prim(double Xf,double V1,double Amax,double Af,double Tjn1,double Jmax,double Tjpb1,double Tjpb2,double X1,double Vf, double V5dc, double A5dc){
  double cg;
  double a,b,c,d,e;
a=A5dc - Jmax * Tjpb2;
b=Tjpb2 + Tjpb1;
c=Tjn1 + Tjpb2;
d=(Amax * Amax - a*a) / 2.0;
e=2.0 * Jmax * Tjpb2 - 2.0 * A5dc;

cg = (-Xf + X1 + V1 * (V5dc - (a) * Tjpb2 - Jmax * Tjpb2 * Tjpb2 / 2.0 - 1.0 / Jmax * d - V1) / Amax + 1.0 / Amax * pow(V5dc - (a) * Tjpb2 - Jmax * Tjpb2 * Tjpb2 / 2.0 - 1.0 / Jmax * d - V1, 2.0) / 2.0 + (V5dc - (a) * Tjpb2 - Jmax * Tjpb2 * Tjpb2 / 2.0 - 1.0 / Jmax * d) * (c) + Amax * c*c / 2.0 - Jmax * c*c*c / 6.0 + (V5dc - (a) * Tjpb2 - Jmax * Tjpb2 * Tjpb2 / 2.0) * (b) + (a) * b*b / 2.0 + Jmax * b*b*b / 6.0) / (V1 * (e) / Amax + 1.0 / Amax * (V5dc - (a) * Tjpb2 - Jmax * Tjpb2 * Tjpb2 / 2.0 - 1.0 / Jmax * d - V1) * (e) + (e) * (c) + 2.0 * V5dc - 2.0 * (a) * Tjpb2 - Jmax * Tjpb2 * Tjpb2 - 1.0 / Jmax * d + Amax * (c) - Jmax * c*c / 2.0 + (Jmax * Tjpb2 - A5dc) * (b) + 2.0 * (A5dc / 2.0 - Jmax * Tjpb2 / 2.0) * (b));
return cg;
}



double l2340prim (double X6,double V6,double Tjpa1,double Tjpa2,double Amax,double Jmax,double Tjn1, double A1dc, double A0, double V0){
  double cg;
  double a,b,c,d,e,f;
a=Tjpa1 + Tjpa2;
b=Tjn1 + Tjpa2;
c=(Amax * Amax - A1dc * A1dc) / 2.0;
d=Jmax * Tjpa2 * Tjpa2 / 2.0;
e=V0 + A0 * (a) + Jmax * a*a / 2.0 + (A0 + Jmax * (a)) * Tjpa2 - d - 1.0 / Jmax * c;
f=e- V6;

cg = (X6 - (e) * (f) / Amax + 1.0 / Amax * f*f / 2.0 - (e- (-Amax + Jmax * (b)) * (b) + Jmax * b*b / 2.0) * (b) - (-Amax + Jmax * (b)) * b*b / 2.0 + Jmax * b*b*b / 6.0 - V0 * (a) - A0 * a*a / 2.0 - Jmax * a*a*a / 6.0) / (-(V0 + A0 * (a) + Jmax * a*a / 2.0 + (A0 + Jmax * (a)) * Tjpa2 - d - 1.0 / Jmax * c) * (2.0 * A0 + 2.0 * Jmax * (a)) / Amax - (2.0 * A0 + 2.0 * Jmax * (a) - Jmax * (b) + Amax) * (b) - 2.0 * V0 - 2.0 * A0 * (a) - Jmax * a*a - (A0 + Jmax * (a)) * Tjpa2 + d + 1.0 / Jmax * c + (-Amax + Jmax * (b)) * (b) - Jmax * b*b / 2.0 - 2.0 * (-Amax / 2.0 + Jmax * (b) / 2.0) * (b));

  return cg;
  }




double l2448prim(double Xf,double X1dc, double V1dc,double Tj1,double A1dc,double Jmax, double Tjdc, double Af, double Amax, double Vf ){
  double cg;
  double a,b,c,d,e,f,g;
a=(2 * Af * Af) - 0.4e1 * Vf * Jmax + 0.8e1 * A1dc * Jmax * Tj1 + 0.4e1 * Jmax * Jmax * Tj1 * Tj1 + 0.4e1 * V1dc * Jmax + 0.2e1 * A1dc * A1dc;
b=(-Jmax * Tjdc +  Af - sqrt( a) / 0.2e1) / Jmax;
c=0.8e1 * A1dc * Jmax + 0.8e1 * Jmax * Jmax * Tj1;
d=b + Tjdc;
e=Tj1 + b;
f=pow( a, -0.1e1 / 0.2e1);
g=A1dc + Jmax * Tj1;

cg = (-Xf + X1dc + V1dc * Tj1 + A1dc * Tj1 * Tj1 / 0.2e1 + Jmax * Tj1*Tj1*Tj1 / 0.6e1 + (V1dc + A1dc * Tj1 + Jmax * Tj1 * Tj1 / 0.2e1) * (e) + (g) * e*e / 0.2e1 - Jmax * e*e*e / 0.6e1 + (V1dc + A1dc * Tj1 + Jmax * Tj1 * Tj1 / 0.2e1 + (g) * (e) - Jmax * e*e / 0.2e1) * (d) + (g - Jmax * (e)) * d*d / 0.2e1 + Jmax * d*d*d / 0.6e1) / (V1dc + A1dc * Tj1 + Jmax * Tj1 * Tj1 / 0.2e1 + (g) * (e) + (V1dc + A1dc * Tj1 + Jmax * Tj1 * Tj1 / 0.2e1) * (0.1e1 - f * (c) / Jmax / 0.4e1) + Jmax * e*e / 0.2e1 + 0.2e1 * (A1dc / 0.2e1 + Jmax * Tj1 / 0.2e1) * (e) * (0.1e1 - f * (c) / Jmax / 0.4e1) - Jmax * e*e * (0.1e1 - f * (c) / Jmax / 0.4e1) / 0.2e1 + (g + Jmax * (e) + (g) * (0.1e1 - f * (c) / Jmax / 0.4e1) - Jmax * (e) * (0.1e1 - f * (c) / Jmax / 0.4e1)) * (d) - (V1dc + A1dc * Tj1 + Jmax * Tj1 * Tj1 / 0.2e1 + (g) * (e) - Jmax * e*e / 0.2e1) * f * (c) / Jmax / 0.4e1 + (Jmax / 0.2e1 - Jmax * (0.1e1 - f * (c) / Jmax / 0.4e1) / 0.2e1) * d*d - (A1dc / 0.2e1 + Jmax * Tj1 / 0.2e1 - Jmax * (e) / 0.2e1) * (d) * f * (c) / Jmax / 0.2e1 - d*d * f * (c) / 0.8e1);
return cg;
}


double l2491prim(double Xf,double X1dc, double V0,double Tj1,double Jmax,double Af, double Vf, double Amax, double A1dc, double V1dc,double Tjdc){
  double cg;
  double a,b,c,d,e,f;
a=Tjdc + Tj1;
b=(2 * Af * Af) - 0.4e1 * Vf * Jmax + 0.8e1 * A1dc * Jmax * Tj1 + 0.4e1 * Jmax * Jmax * Tjdc * Tjdc + 0.4e1 * V1dc * Jmax + 0.8e1 * A1dc * Jmax * Tjdc + 0.2e1 * A1dc * A1dc + 0.8e1 * Jmax * Jmax * Tjdc * Tj1 + 0.4e1 * Jmax * Jmax * Tj1 * Tj1;
c=Af - sqrt( b) / 0.2e1;
d=Tj1 + ( c) / Jmax;
e=pow( b, -0.1e1 / 0.2e1);
f=pow(Jmax, -0.2e1);

cg = (-Xf + X1dc + V1dc * (a) + A1dc * a*a / 0.2e1 + Jmax * a*a*a / 0.6e1 + (V1dc + A1dc * (a) + Jmax * a*a/ 0.2e1) * (d) + (A1dc + Jmax * (a)) * d*d / 0.2e1 - Jmax * d*d*d / 0.6e1 + (V1dc + A1dc * (a) + Jmax * a*a / 0.2e1 + (A1dc + Jmax * (a)) * (d) - Jmax * d*d / 0.2e1) * ( c) / Jmax + (A1dc + Jmax * (a) - Jmax * (d)) * c*c * f / 0.2e1 + f * c*c*c / 0.6e1) / (V1dc + A1dc * (a) + Jmax * a*a / 0.2e1 + (A1dc + Jmax * (a)) * (d) + (V1dc + A1dc * (a) + Jmax * a*a / 0.2e1) * (0.1e1 - e * (0.8e1 * A1dc * Jmax + 0.8e1 * Jmax * Jmax * Tjdc + 0.8e1 * Jmax * Jmax * Tj1) / Jmax / 0.4e1) + Jmax * d*d / 0.2e1 + 0.2e1 * (A1dc / 0.2e1 + Jmax * (a) / 0.2e1) * (d) * (0.1e1 - e * (0.8e1 * A1dc * Jmax + 0.8e1 * Jmax * Jmax * Tjdc + 0.8e1 * Jmax * Jmax * Tj1) / Jmax / 0.4e1) - Jmax * d*d * (0.1e1 - e * (0.8e1 * A1dc * Jmax + 0.8e1 * Jmax * Jmax * Tjdc + 0.8e1 * Jmax * Jmax * Tj1) / Jmax / 0.4e1) / 0.2e1 + (A1dc + Jmax * (a) + Jmax * (d) + (A1dc + Jmax * (a)) * (0.1e1 - e * (0.8e1 * A1dc * Jmax + 0.8e1 * Jmax * Jmax * Tjdc + 0.8e1 * Jmax * Jmax * Tj1) / Jmax / 0.4e1) - Jmax * (d) * (0.1e1 - e * (0.8e1 * A1dc * Jmax + 0.8e1 * Jmax * Jmax * Tjdc + 0.8e1 * Jmax * Jmax * Tj1) / Jmax / 0.4e1)) * ( c) / Jmax - (V1dc + A1dc * (a) + Jmax * a*a / 0.2e1 + (A1dc + Jmax * (a)) * (d) - Jmax * d*d / 0.2e1) * e * (0.8e1 * A1dc * Jmax + 0.8e1 * Jmax * Jmax * Tjdc + 0.8e1 * Jmax * Jmax * Tj1) / Jmax / 0.4e1 + (Jmax / 0.2e1 - Jmax * (0.1e1 - e * (0.8e1 * A1dc * Jmax + 0.8e1 * Jmax * Jmax * Tjdc + 0.8e1 * Jmax * Jmax * Tj1) / Jmax / 0.4e1) / 0.2e1) * c*c * f - (A1dc / 0.2e1 + Jmax * (a) / 0.2e1 - Jmax * (d) / 0.2e1) * ( c) * f * e * (0.8e1 * A1dc * Jmax + 0.8e1 * Jmax * Jmax * Tjdc + 0.8e1 * Jmax * Jmax * Tj1) / 0.2e1 - f * c*c * e * (0.8e1 * A1dc * Jmax + 0.8e1 * Jmax * Jmax * Tjdc + 0.8e1 * Jmax * Jmax * Tj1) / 0.8e1);
return cg;
}



double l2535prim(double Xf,double X1dc, double V0,double Tj1,double Jmax,double Af, double Vf, double Amax, double A1dc, double V1dc,double Tjdc){
  double cg;
  double a,b,c,d,e,f,g,h,i;

a=Jmax * Tj1;
  b=A1dc + a;
  c=V1dc + A1dc * Tj1 + a * Tj1 / 2.0;
  d=(2 * Af * Af) - 4.0 * Vf * Jmax + 4.0 * Jmax * a * Tj1 + 4.0 * V1dc * Jmax + 8.0 * A1dc * a + 2.0 * A1dc * A1dc;
  e=8.0 * Jmax * a + 8.0 * A1dc * Jmax;
  f=Af - sqrt( d) / 2.0;
  g=Tj1 + ( f) / Jmax + Tjdc;
  h=pow(Jmax, -2.0);
  i=pow( d, -1.0 / 2.0);

cg = (-Xf + X1dc + V1dc * Tj1 + A1dc * Tj1 * Tj1 / 2.0 + Jmax * Tj1*Tj1*Tj1 / 6.0 + (c) * (g) + (b) * g*g / 2.0 - Jmax * g*g*g / 6.0 + (c + (b) * (g) - Jmax * g*g / 2.0) * ( f) / Jmax + (b - Jmax * (g)) * f*f * h / 2.0 + h * f*f*f / 6.0) / (c + (b) * (g) + (c) * (1.0 - i * (e) / Jmax / 4.0) + Jmax * g*g / 2.0 + 2.0 * (A1dc / 2.0 + a / 2.0) * (g) * (1.0 - i * (e) / Jmax / 4.0) - Jmax * g*g * (1.0 - i * (e) / Jmax / 4.0) / 2.0 + (b + Jmax * (g) + (b) * (1.0 - i * (e) / Jmax / 4.0) - Jmax * (g) * (1.0 - i * (e) / Jmax / 4.0)) * ( f) / Jmax - (c + (b) * (g) - Jmax * g*g / 2.0) * i * (e) / Jmax / 4.0 + (Jmax / 2.0 - Jmax * (1.0 - i * (e) / Jmax / 4.0) / 2.0) * f*f * h - (A1dc / 2.0 + a / 2.0 - Jmax * (g) / 2.0) * ( f) * h * i * (e) / 2.0 - h * f*f * i * (e) / 8.0);

return cg;
}
