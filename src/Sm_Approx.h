#ifndef SM_APPROX_H
#define SM_APPROX_H

#include "softMotionStruct.h"
#include "softMotion.h"
#include "Sm_Traj.h"
#include <string>
#include "Sm_Curve.h"




class Sm_Approx {
 public:
    /** @brief Constructor
     *
     * The constructor of class QSoftMotionPlanner
     *
     * @param *parent : Null
     */
    Sm_Approx();
    /** @brief Destructor
     *
     * The destructor of class QSoftMotionPlanner
     */
   ~Sm_Approx();


    /** @brief compute the approximation of a given svg file
     *
     * @param jmax : maximum jerk (m/s^3)
     * @param amax : maximum acceleration (m/s^2)
     * @param vmax : maximum velocity (m/s)
     * @param sampTime : discretisation time (second)
     * @param ErrMax : maximum error of the trajectory allowed
     * @param ExpTime : discretisation of the generated file
     * @param flagExport : genere the result file (true: yes, false: no)
     * @param fileName : the svg file to approximate
     */
    void approximate(double jmax,double amax,double vmax,double sampTime, double ErrMax, int ExpTime, 
		     bool flagExport, std::string fileName);

    void approximate(Sm_Curve curv, double SampTime,  double ErrMax,int ExpTime, bool flagExport, std::string fileName);


    void approximate(double jmax,double amax,double vmax,double sampTime, double ErrMax, int ExpTime, 
		     bool flagExport, std::string fileName, SM_TRAJ &traj);

    /** @brief load the file
     *
     * load a file .svg
     *
     * @param str : file name
     */
    void loadSvgFile(std::string str);



    void maxProfile(std::vector<SM_CURVE_DATA>  &ApproxTraj, double *max_jerk, double *max_acc, double *max_vel);
    void initializeApproxVariables();
    void computeTraj();

    void computeHausdorff();


    void resetPlanner();


    /** @brief generate a data file
     *
     * export the discretized approximated trajectory into a file
     * with a specified sampling time ( 10 ms by default)
     */
    void genFileTraj();

    void genPlotFile();


    void setNbAxis(int v);

private:
    int _nbCurve;
    int _nbAxis;
    int _flag_haus_actif;
    std::vector<Sm_Curve> _curve; // stocker la trajectoire ideale et approxime
    std::vector<kinPoint> _vec_kinpoint;// les points pour exporter dans le fichier SmDiscr.dat
    std::vector<double> _err_traj;
    std::vector<double> _err_vit;
    std::vector<double> _err_haus1;
    std::vector<double> _err_haus2;
    std::vector<double> _courbure;
    std::vector<double> _vec_interval_courbure;
//     std::vector<double> _t_Mlaw,_u_Mlaw,_du_Mlaw,_ddu_Mlaw;
    SM_LIMITS _lim;
    std::vector<SM_OUTPUT> _result;
    double _errMax;
    double _rayon_circle_for_courbure;
    double _interval_courbure;
    double _sampling;// temps d'echantionnage
    int _timeStep;// interval pour generer le fichier
     std::string _fileName;
 
 
};

#endif
