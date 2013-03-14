#include <cstdlib>
#include <string>
#include <iostream>
#include <vector>

#include "Sm_Approx.h"

using namespace std;

const float SAMPLING_TIME = 0.005;

void usage(char* name) {
    cout << "Usage: " << name << "  <path.svg>" << endl << endl;
    cout << "Reads the first path from a SVG file and output " << endl;
    cout << "on stdout the corresponding softMotion trajectory." << endl;
}

int main(int argc, char *argv[]) {

    if (argc != 2) {
        usage(argv[0]);
        exit(1);
    }

     string      svgFile = string(argv[1]);

     double      jmax=0.9;
     double      amax=0.3;
     double      vmax=0.3;
     double      ErrMax=0.005;
     int         subsampling=1;

     Sm_Approx approx = Sm_Approx();

     approx.setNbAxis(2);
     approx.approximate(jmax, amax, vmax, SAMPLING_TIME, ErrMax, subsampling, false, svgFile);

     return 0;

}
