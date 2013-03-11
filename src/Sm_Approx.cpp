
#include "Sm_Approx.h"
#include <iostream>
#include "time_proto.h"


using namespace std;

Sm_Approx::Sm_Approx()
{
  _nbCurve = 0;
  _nbAxis = 0;
  _fileName = "";
  _curve.clear();
}

Sm_Approx::~Sm_Approx()
{
  resetPlanner();
}

void Sm_Approx::approximate(double jmax,double amax,double vmax,double sampTime, double ErrMax, 
			    int ExpTime, bool flagExport, std::string fileName, SM_TRAJ &traj)
{
  this->approximate(jmax, amax, vmax, sampTime,  ErrMax,  ExpTime, flagExport, fileName);
  traj.clear();


  return;
}


// fonction d'interface avec SM_TRAJ
int Sm_Approx::approximate(Sm_Curve &curv, double SampTime, double ErrPosMax, double ErrVelMax, SM_TRAJ &smTraj, bool flag)
{
  std::string fileName;
  fileName.clear();
  fileName.append("approx.traj");
  bool flagExport= flag;
  int ExpTime = SampTime;
  int res= approximate(curv, SampTime,  ErrPosMax, ErrVelMax, ExpTime, flagExport, fileName, smTraj);
  return res;
}


int Sm_Approx::approximate(Sm_Curve &curv, double SampTime,  double ErrPosMax, double ErrVelMax, int ExpTime, bool flagExport, std::string fileName, SM_TRAJ &smTraj)
{
  std::string str2;
  FILE *fp_segMotion = NULL;
  _sampling = SampTime;
  _errMax = ErrPosMax;
  _timeStep = ExpTime;
  _flag_haus_actif = 0;
  _fileName = fileName;
  _nbAxis = curv.traj[0].Pos.size() ;
  SM_OUTPUT tempo_motion;
     double tu, ts;


  /* set limits that are only used to compute the error in velocity on overall the trajectory */
  _lim.maxJerk = 6*fabs(ErrVelMax);
  _lim.maxAcc  = 2*fabs(ErrVelMax);
  _lim.maxVel  = fabs(ErrVelMax);
   
  initializeApproxVariables();

  // dans l'attribut _curve il y aura deux courbes, la premiere _curve[0] est la traj ideale
  // la second _curve[1] = (_curve.back()) sera la traj approximee
  // les suites de cubiques de la traj approxime sont stockées dans _result
  // _result.size() est la nombre de segment de cubique (c'est la meme pour tout les axes)

  // SAUVE LA TRAJCTOIRE IDEALE (sous la forme d'un tableau)
  if(flagExport==true) {
    str2.clear();
    str2 += "SmIdealTraj.dat";
    saveTraj(str2, curv.traj);
    LOG(INFO, "Sm_Approx::approximate Ideal Trajectory Already Computed and Saved");
  }

  /* Handle the path */
  _curve.push_back(curv);

  ChronoOn();
  ChronoTimes(&tu, &ts);
  /// CALCUL DE L'APPROXIMATION //
  computeTraj();
  LOG(INFO, "Sm_Approx::approximate Computation Duration : ");
  ChronoPrint("");
  std::cout << std::endl;
  ChronoTimes(&tu, &ts);
  ChronoOff();


  if(flagExport==true) {
    // SAUVE LA TRAJCTOIRE APPROXIMEE (sous la forme d'un tableau)
    saveTraj("SmApproxTraj.dat", (_curve.back()).traj);
    LOG(INFO, "Sm_Approx::approximate Approximated Trajectory Saved");
  }

  LOG(INFO, "Sm_Approx::approximate Error Max Pos " << (_curve.back()).errorMaxVal);

  /* fill result */
  for (unsigned int i = 1; i < _result.size(); i++){
    for (unsigned int j = 0; j < _result.size()-i ; j++){
      if (_result[j].premier_point > _result[j+1].premier_point){
        tempo_motion = _result[j];
        _result[j] = _result[j+1];
        _result[j+1] = tempo_motion;
      }
    }
  }

  _result[0].IC.resize(_result[0].Time.size());

  for(unsigned int i=0; i<_result[0].Time.size(); i++) {
    _result[0].IC[i].x = curv.traj[0].Pos[i];
  }

  if(flagExport==true) {
    //printf("approximate: There are %f axes and %f segments\n", (double)_result[0].Time.size(), (double)_result.size());  
    fp_segMotion = fopen("SmSegMotion.dat", "w");
    if(fp_segMotion==NULL) {
      LOG(ERROR, " cannot open file to write the trajectory");
      return 1;
    }
    for (unsigned int i = 0; i < _result.size(); i++){
      fprintf(fp_segMotion, "%d %lf ",_result[i].premier_point, (_result[i].premier_point + 1) * _sampling);
      for(unsigned int k=0; k<_result[i].Time.size(); k++) {
	fprintf(fp_segMotion, "%lf %lf ",_result[i].Time[k],_result[i].Jerk[k]);
      }
      fprintf(fp_segMotion, "\n");
    }
    fclose(fp_segMotion);
    //cout << "motion file exported" << endl;
    
    //printf("approximate: There are %f axes and %f segments\n", (double)_result[0].Time.size(), (double)_result.size());
  }

  // importe le resultat _result dans une classe de type SM_TRAJ Attention on recalcule tte les conditions initiales
  // de chaque segment dans cette fonction ainsi que la duree totale de la trajectoire a partir des couples (Ti, Ji)
  smTraj.importFromSM_OUTPUT(36, _sampling, _result);

  //cout << " ... Approximated Trajectory Computed --" << endl;

  if(flagExport==true) {  
    smTraj.save((char*)"SmApproxTraj_Seg.traj");
    LOG(INFO, "Sm_Approx::approximate Series of cubics Trajectory Saved");
  }
  /* compare end point */
  /* verification du point final atteint par smTraj par rapport a IdealTraj */
  std::vector< SM_COND> cond;
  smTraj.getMotionCond(smTraj.getDuration(), cond);
  double errMax =0;
  for(unsigned int i=0; i<cond.size(); i++) {
    double err = fabs(cond[i].x -  (_curve.back()).traj[ (_curve.back()).traj.size()-1].Pos[i] );
    if( err > 0.001) {
        LOG(WARNING, "Sm_Approx::approximate: final pose on axis " << i << ", err=" << err);
    }
    if(err > errMax){
      errMax = err;
    }
  }
  if(errMax < 0.001) {
    LOG(INFO, "Sm_Approx::approximate runs successfully ... Algo Written by Xavier BROQUERE (Feb 2011)");
  }
  //printf("Final Conditions of the Ideal trajectory\n");
  //for(unsigned int i=0; i<cond.size(); i++) {
  //  printf("%f ",(_curve.back()).traj[ (_curve.back()).traj.size()-1].Pos[i] );
  //}
  //printf("\n"); 
  //printf("Final Conditions of the smTraj trajectory\n");
  //for(unsigned int i=0; i<cond.size(); i++) {
  //  printf("%f ",cond[i].x);
  //}
  //printf("\n");

  return 0;
}


void Sm_Approx::approximate(double jmax,double amax,double vmax,double SampTime, double ErrMax, 
			    int ExpTime, bool flagExport, std::string fileName) {

  FILE *fp_segMotion = NULL;
  double tu = 0.0,ts = 0.0;
  _lim.maxJerk = jmax;
  _lim.maxAcc  = amax;
  _lim.maxVel  = vmax;
  _sampling = SampTime;
  _errMax = ErrMax;
  _timeStep = ExpTime;
  _flag_haus_actif = 0;
  _fileName = fileName;
  SM_OUTPUT tempo_motion;

  std::string str;
  std::string str2;
  Sm_Curve curv;

 
  if(_nbAxis == 0){
    LOG(ERROR, "NbAxis is null, set it before with setNbAxis() ");
    return;
  }
  LOG(INFO, "Open file " << this->_fileName);

  if (parseSvg(this->_fileName.c_str(), curv.path, &curv.width, &curv.height) == SM_ERROR) {
    LOG(INFO, "... file parsed ");
    return;
  }

  initializeApproxVariables();
  constructTrajSvg(curv.path, _sampling, _lim, curv.traj);
  str2.clear();
  str2 += "QtIdealTraj.dat";
  saveTraj(str2, curv.traj);

  /* Handle the path */
  _curve.push_back(curv);
  LOG(INFO, " ... Ideal Trajectory Computed ");

  ChronoOn();

  computeTraj();
  LOG(INFO, " ... Approximated Trajectory Computed --> Algo Written by Xavier BROQUERE");


  if(flagExport) {
    genFileTraj(false);
    LOG(INFO, " ... File exported ");
  }
  else { // output it on stdout
      LOG(INFO, "Writing approximated trajectory on stdout...");
      genFileTraj(true);
  }

  /* fill result */
  for (unsigned int i = 1; i < _result.size(); i++){
    for (unsigned int j = 0; j < _result.size()-i ; j++){
      if (_result[j].premier_point > _result[j+1].premier_point){
        tempo_motion = _result[j];
        _result[j] = _result[j+1];
        _result[j+1] = tempo_motion;
      }
    }
  }

  fp_segMotion = fopen("segMotion.dat", "w");
  if(fp_segMotion==NULL) {
    LOG(ERROR, " cannont open file to write the trajectory");
    return;
  }

  for (unsigned int i = 0; i < _result.size(); i++){
    fprintf(fp_segMotion, "%d %lf %lf %lf %lf %lf %lf %lf\n", _result[i].premier_point, 
	    (_result[i].premier_point + 1) * _sampling, 
	    _result[i].Jerk[0], _result[i].Jerk[1], _result[i].Jerk[2], 
	    _result[i].Time[0], _result[i].Time[1], _result[i].Time[2]);
  }

  fclose(fp_segMotion);
  LOG(INFO, "motion file exported");

  ChronoTimes(&tu, &ts);
  ChronoPrint("");
  ChronoOff();
  return ;
}



void Sm_Approx::maxProfile(std::vector<SM_CURVE_DATA>  &ApproxTraj, double *max_jerk, double *max_acc, double *max_vel){
  double v = 0.0;
  double a = 0.0;
  double j = 0.0;
  std::vector<double> vel_norm_vec;
  std::vector<double> acc_norm_vec;
  std::vector<double> jerk_norm_vec;
  for(unsigned int i = 0; i < ApproxTraj.size(); i++){
    v = sqrt(
	     ApproxTraj[i].Vel[0] * ApproxTraj[i].Vel[0] +
	     ApproxTraj[i].Vel[1] * ApproxTraj[i].Vel[1] +
	     ApproxTraj[i].Vel[2] * ApproxTraj[i].Vel[2]
	     );
    a = sqrt(
	     ApproxTraj[i].Acc[0] * ApproxTraj[i].Acc[0] +
	     ApproxTraj[i].Acc[1] * ApproxTraj[i].Acc[1] +
	     ApproxTraj[i].Acc[2] * ApproxTraj[i].Acc[2]
	     );
    j = sqrt(
	     ApproxTraj[i].Jerk[0] * ApproxTraj[i].Jerk[0] +
	     ApproxTraj[i].Jerk[1] * ApproxTraj[i].Jerk[1] +
	     ApproxTraj[i].Jerk[2] * ApproxTraj[i].Jerk[2]
	     );
    jerk_norm_vec.push_back(j);
    acc_norm_vec.push_back(a);
    vel_norm_vec.push_back(v);
  }

  *max_vel = vel_norm_vec[0];
  *max_acc = acc_norm_vec[0];
  *max_jerk = jerk_norm_vec[0];
  for(unsigned int i = 0; i < ApproxTraj.size(); i++){
    if(*max_vel < vel_norm_vec[i])     { *max_vel = vel_norm_vec[i]; }
    if(*max_acc < acc_norm_vec[i])     { *max_acc = acc_norm_vec[i]; }
    if(*max_jerk < jerk_norm_vec[i])   { *max_jerk = jerk_norm_vec[i]; }
  }

  
}

void Sm_Approx::initializeApproxVariables(){
  this->_curve.clear();
}


void Sm_Approx::genPlotFile(){

  int incr = _timeStep;
  FILE *fp_SmCurves = NULL;
  FILE *fp_SmDiscr = NULL;
  FILE *fp_SmMaxError = NULL;
  fp_SmCurves = fopen("SmCurves.dat", "w");
  fp_SmDiscr = fopen("SmDiscr.dat", "w");
  fp_SmMaxError = fopen("SmMaxError.dat", "w");
  if((fp_SmCurves==NULL)||(fp_SmDiscr==NULL)||(fp_SmMaxError==NULL)) {
    std::cerr << " cannont open file to write the trajectory" << std::endl;
    return;
  }
  if (_flag_haus_actif == 0){
    _err_haus1.resize(_curve.back().traj.size());
    _err_haus2.resize(_curve.back().traj.size());
  }

  //   Calcul_Error_Vilocity(_curve.front().traj, _curve.back().traj, _err_vit, &errMax_pos_subTraj_vit);

  for (unsigned int i = 0; i < _curve.back().traj.size(); i += incr){
    fprintf(fp_SmCurves, "%d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n", i, i*_sampling, _curve.front().traj[i].Pos[0], _curve.front().traj[i].Pos[1], _curve.front().traj[i].Pos[2], _curve.front().traj[i].Vel[0], _curve.front().traj[i].Vel[1], _curve.front().traj[i].Vel[2], _curve.front().traj[i].Acc[0], _curve.front().traj[i].Acc[1], _curve.front().traj[i].Acc[2], _curve.back().traj[i].Pos[0], _curve.back().traj[i].Pos[1], _curve.back().traj[i].Pos[2], _curve.back().traj[i].Vel[0], _curve.back().traj[i].Vel[1], _curve.back().traj[i].Vel[2], _curve.back().traj[i].Acc[0], _curve.back().traj[i].Acc[1], _curve.back().traj[i].Acc[2], _err_traj[i],  (_curve.front().traj[i].du-_curve.back().traj[i].du), _err_haus1[i], _err_haus2[i],  _curve.front().traj[i].ddu_Mlaw,  _curve.front().traj[i].du_Mlaw,  _curve.front().traj[i].u_Mlaw, _curve.front().traj[i].ddu, _curve.front().traj[i].du, _curve.front().traj[i].u, _curve.back().traj[i].ddu, _curve.back().traj[i].du, _curve.back().traj[i].u);
  }
  fclose(fp_SmCurves);

  for (unsigned int i = 0; i < _vec_kinpoint.size(); i ++){
    fprintf(fp_SmDiscr, "%d %lf %lf %lf %lf\n", 1+int(_vec_kinpoint[i].t/_sampling), _vec_kinpoint[i].t, _vec_kinpoint[i].kc[0].x, _vec_kinpoint[i].kc[1].x, _vec_kinpoint[i].kc[2].x);
  }
  fclose(fp_SmDiscr);

  fprintf(fp_SmMaxError, "%d %lf %lf %lf %lf\n", 1 + int(_curve.front().errorMax.t/_sampling), _curve.front().errorMax.t,  _curve.front().errorMax.kc[0].x, _curve.front().errorMax.kc[1].x, _curve.front().errorMax.kc[2].x);
  fclose(fp_SmMaxError);

  // SmDiscr index t x y z
  // SmMaxError index t x y z
  return;
}

void Sm_Approx::setNbAxis(int v)
{
  _nbAxis = v;
}

void Sm_Approx::genFileTraj(bool use_stdout){

  int incr = _timeStep;

  FILE *fp = NULL;

  if (use_stdout) fp = stdout;
  else fp = fopen("output.traj", "w");

  if(fp==NULL) {
    LOG(ERROR, " canont open file to write the trajectory");
    return;
  }

  LOG(INFO, "Number of positions in the trajectory: " <<_curve.back().traj.size());
  LOG(INFO, "Subsampling level: " << incr);
  LOG(INFO, "Number of resulting positions: " << (int) _curve.back().traj.size() / incr);
  for (unsigned int i = 0; i < _curve.back().traj.size(); i += incr){
    fprintf(fp, "%lf\t", _curve.back().traj[i].Pos[0]);
    fprintf(fp, "%lf\t", _curve.back().traj[i].Pos[1]);
    fprintf(fp, "%lf\n", _curve.back().traj[i].Pos[2]);
  }
  fclose(fp);

  return;
}

void Sm_Approx::computeTraj(){
  SM_LIMITS lim;
  SM_OUTPUT outMotion;
  int premier_point_motionSeg;
  int nbIntervals_local = 0;
  int kkk = 0;
  int hh = 0;
  int nouveau_temp_divis = 0; //counter for the sub-trajectory
  int size_segment = 0; //size of each sub-trajectory
  int flag_sum = 0; //it shows whether we need to divise a sub-trajectory
  int starting_point_each_seg = 0; // the starting points of each sub-trajectory
  int nb_seg_total = 0;
  int test_for_circle_only = 0;
  //  double tu = 0.0,ts = 0.0; // chrono on et off
  double tic = 0.0;
  double time_total = 0.0;
  double errMax_pos_subTraj = 0.0; //maxi error of each sub-trajectory
  double errMax_vel_subTraj = 0.0;
  double errMax_pos_traj = 0.0;//maxi error of the whole trajectory

  double err_max_def_pos = 0.0;
  double err_max_def_vel = 0.0;
  double err_max_pos_chaq_seg = 0.0;
  double err_max_vel_chaq_seg = 0.0;
  double max_acc = 0.0;
  double max_vel = 0.0;
  double max_jerk = 0.0;
  kinPoint kc_subTraj;

  Sm_Curve curv2; //approximated curve
  Sm_Curve curv_temp; //ideal curve for the iteration
  Sm_Curve curv_divis; //approximated and divised curve
  Sm_Curve curv_temp_divis; //ideal and divised curve
  Sm_Curve curv_stock; // curve for the divised trajectory
  Sm_Curve curv_temp_cond; //curve just for calculate the IC and FC

  std::vector<SM_COND_DIM> IC;
  std::vector<SM_COND_DIM> FC;
  std::vector<double> Timp;
  std::vector<int> IntervIndex;
  std::vector<SM_OUTPUT> motion;
  std::vector<double> error;
  std::vector<double> error_vel;
  std::list<SubTraj>::iterator iter_temp;
  std::list<SubTraj>::iterator iter_stock;
  ChronoOn();

  kc_subTraj.kc.resize(_nbAxis);

  lim.maxJerk = _lim.maxJerk;
  lim.maxAcc = _lim.maxAcc;
  lim.maxVel = _lim.maxVel;
  tic = _sampling;
  err_max_def_pos = _errMax;
  err_max_def_vel = lim.maxVel;

  //   Path_Length(_curve.front().path, &longeur_path);

  //  saveTraj("QtIdealTraj2.dat", _curve.begin()->traj);
 
  time_total = (_curve.front().traj.size()-1) * tic;

  curv2.traj.clear();
  curv2.traj.resize(_curve.front().traj.size());
  nbIntervals_local = 1; /* at the beginning there is only 1 subTraj */
  error.clear();
  error_vel.clear();
  errMax_pos_subTraj = 0.0;
  SM_CURVE_DATA curv_donne;
  std::vector<double> Temp_alias(1);
  curv_temp_divis.trajList.resize(nbIntervals_local);
  curv_temp_divis.trajList.front().traj.resize(_curve.front().traj.size());
  curv_temp_cond.trajList.resize(1);
  std::list<SubTraj>::iterator iter_temp_cond;
  std::list<SubTraj>::iterator iter_temp_divis;


  /* copy the entire ideal curve into the subtraj list:
     at the beginning there is only one subTraj */
  for (iter_temp_divis = curv_temp_divis.trajList.begin(); iter_temp_divis != curv_temp_divis.trajList.end(); iter_temp_divis ++){
    iter_temp_divis->point_depart = 0;
    iter_temp_divis->traj = _curve.front().traj;
  }

  do{
    /****************************/
    /* initialize the variables */
    /****************************/
    flag_sum = 0;
    nouveau_temp_divis = 0;
    std::list<SubTraj>::iterator iter_divis;
    curv_stock.trajList.resize(nbIntervals_local*2);
    curv_temp_cond.trajList.resize(curv_temp_divis.trajList.size());
    for(iter_stock = curv_stock.trajList.begin(); iter_stock != curv_stock.trajList.end(); iter_stock ++){
      iter_stock->traj.clear();
      iter_stock->flag_traj = -1;
    }
    for(iter_temp_cond = curv_temp_cond.trajList.begin(); iter_temp_cond != curv_temp_cond.trajList.end(); iter_temp_cond ++){
      iter_temp_cond->traj.clear();
    }
    iter_stock = curv_stock.trajList.begin();
    iter_temp_cond=curv_temp_cond.trajList.begin();

    /************************************************/
    /* copy the curv_temp_divis into curv_temp_cond */
    /************************************************/
    for (iter_temp_divis = curv_temp_divis.trajList.begin(); iter_temp_divis != curv_temp_divis.trajList.end(); iter_temp_divis ++){
      for(unsigned int kk = 0; kk < iter_temp_divis->traj.size(); kk ++){
        curv_donne = iter_temp_divis->traj[kk];
        iter_temp_cond->traj.push_back(curv_donne);
      }
      iter_temp_cond ++;
    }

    iter_temp_divis = curv_temp_divis.trajList.begin();
    curv_divis.trajList.resize(nbIntervals_local);
    iter_divis = curv_divis.trajList.begin();
    for(iter_temp_cond=curv_temp_cond.trajList.begin(); iter_temp_cond != curv_temp_cond.trajList.end(); iter_temp_cond++){
      kkk = 0;
      err_max_pos_chaq_seg = 0.0;
      err_max_vel_chaq_seg = 0.0;
      IC.resize(1);
      IC[0].Axis.resize(_nbAxis);
      FC.resize(1);
      FC[0].Axis.resize(_nbAxis);
      Timp.resize(1);
      IntervIndex.resize(2);

      if (sm_ComputeCondition(iter_temp_cond->traj, _curve.front().discPoint, IC, FC, Timp, IntervIndex) != 0){
	printf("Compute Problem \n");   // ici discpoint marche pas !
	return;
      }


      std::vector<SM_COND_DIM> IC_seg(1);
      std::vector<SM_COND_DIM> FC_seg(1);
      
      IC_seg[0].Axis.resize(_nbAxis);
      FC_seg[0].Axis.resize(_nbAxis);

      error.clear();
      error_vel.clear();
      IC_seg[0].Axis = IC[hh].Axis;
      FC_seg[0].Axis =  FC[hh].Axis;
      //memcpy(IC_seg[0].Axis, IC[hh].Axis, sizeof(SM_COND_DIM));
      //memcpy(FC_seg[0].Axis, FC[hh].Axis, sizeof(SM_COND_DIM));

      iter_divis->motion_par_seg.resize(3);
      Temp_alias.clear();

/// HERRRREEEEEE
      Temp_alias.push_back((iter_temp_divis->traj.size()-1)*tic);
      //Temp_alias.push_back((iter_temp_divis->traj.size())*tic);



 //       /***********************************/
      /* approximate the current subTraj */
      /***********************************/
      // printf("temp alias %f \n", Temp_alias[0]);
      sm_SolveWithoutOpt(IC_seg, FC_seg, Temp_alias, iter_divis->motion_par_seg);
      iter_divis->traj.clear();
      convertMotionToCurve2(iter_divis->motion_par_seg,_nbAxis, tic, 1, iter_divis->traj);

//       printf("segment\n");
      // printf("T1 %f \n", iter_divis->motion_par_seg[0].Time[0]);

      /****************************************/
      /* compute error of the current subTraj */
      /****************************************/
      // iter_temp_divis->traj --> the ideal traj
      // iter_divis->traj --> the approximated traj
      Calcul_Error_list(iter_temp_divis->traj, iter_divis->traj, &_curve.front().errorMax, error, error_vel, &errMax_pos_subTraj, &errMax_vel_subTraj);
      //printf("Errerur %f \n",errMax_pos_subTraj);
//printf("##########################################\n");



      if ( ( (errMax_pos_subTraj > err_max_def_pos)|| (errMax_vel_subTraj > err_max_def_vel) )  && (iter_temp_divis->traj.size()> 12)){ 

	// !!!!!!!!!!!!!!! if the subtraj length is less than 12*_sampling, accept the error !!!!!!!!!!!!!!!!!!!
	
	/* the trajectory error is too high */
	for (int i = 0; i < 2; i++){
	  if(i == 0){
	    size_segment = int((iter_temp_divis->traj.size()-1)/6)*3 +1;

	    //printf("size segment%d %d \n", i, size_segment);

	
	    if(size_segment < 3) {
	      printf("ERROR size_segment < 3\n");
	    }
	    iter_stock->point_depart = iter_temp_divis->point_depart;
	  }
	  else{
	    iter_stock->point_depart = iter_temp_divis->point_depart + size_segment -1;
	    size_segment = iter_temp_divis->traj.size() - size_segment +1 ;
	    //printf("size segment%d %d mult %f\n", i, size_segment, (double)size_segment/3.0);
	  }



	  /* recopy the ideal subTraj */
	  for (int k=0; k< size_segment; k++) {
	    curv_donne = iter_temp_divis->traj[kkk];
	    iter_stock->traj.push_back(curv_donne);
	    kkk ++;
	  }
	  // bug here add kkk--
	  kkk--;
	  /* iter_stock->flag_traj = 1 --> the error is BAD */
	  iter_stock->flag_traj = 1;
	  iter_stock++;
	  nouveau_temp_divis ++;
	}

      }
      else{
	if( (errMax_pos_subTraj > err_max_def_pos)){
	  LOG(WARNING, "Cannot approximate the subtraj accept the error " << errMax_pos_subTraj);
	}
	/* save the approximated subTraj into curv_stock */
	if(test_for_circle_only == 0){
	  _interval_courbure = 2 * M_PI * _rayon_circle_for_courbure / nbIntervals_local ;
	  test_for_circle_only = 1;
	}
	iter_stock->point_depart = iter_temp_divis->point_depart;
	premier_point_motionSeg = iter_temp_divis->point_depart;
	size_segment = iter_divis->traj.size();


	for(int nb_motionSeg = 0; nb_motionSeg < 3; nb_motionSeg ++){ // here 3 represent the 3 segments
	 
	  outMotion.premier_point = premier_point_motionSeg + nb_motionSeg * int(iter_divis->motion_par_seg[nb_motionSeg].Time[0]/(double)_sampling);
	  outMotion.Jerk = iter_divis->motion_par_seg[nb_motionSeg].Jerk;
	  outMotion.Time = iter_divis->motion_par_seg[nb_motionSeg].Time;

          //Xav add copy IC
	  outMotion.IC = iter_divis->motion_par_seg[nb_motionSeg].IC;
	  
	  //  printf("motionseg %d outMotion.premier_point %f time %f \n", nb_motionSeg, outMotion.premier_point, (double)(outMotion.Time[0] ));

	  for(unsigned int ind = 0; ind<outMotion.Jerk.size(); ind++) {
	    if(isnan(outMotion.Jerk[ind]) || isnan(outMotion.Time[ind])) {
	      printf("ERROR isnan on axis %d jerk %f time %f\n",(int)ind, outMotion.Jerk[ind], outMotion.Time[ind]);
	    }
	  }
	  if(outMotion.Time[0]< _sampling) {
	    printf("time < _sampling on axis \n");
	  }
	  _result.push_back(outMotion);
	}
	for (int k = 0; k < size_segment; k++){
	  curv_donne = iter_divis->traj[k];
	  iter_stock->traj.push_back(curv_donne);
	}

	/* iter_stock->flag_traj = 0 the error is OK */
	iter_stock->flag_traj = 0;
	iter_stock++;
      }
      iter_divis++;
      iter_temp_divis ++;
    } /* for all subTraj in curv_temp_cond */

    /**********************************************************************************/
    /* here one loop of the trajectory approximation is done --> saved in curv_stock  */
    /*in curv_stock there are approximated and ideal (where error is too high) subTraj*/
    /**********************************************************************************/

    /* re-initialize the variables */
    flag_sum = 0;
    nbIntervals_local = nouveau_temp_divis;
    curv_temp_divis.trajList.resize(nbIntervals_local);
    for(iter_temp_divis = curv_temp_divis.trajList.begin(); iter_temp_divis != curv_temp_divis.trajList.end(); iter_temp_divis ++){
      iter_temp_divis->traj.clear();
    }
    iter_temp_divis = curv_temp_divis.trajList.begin();


    /*****************************************************************/
    /* recopy in curv_temp_divis the subTraj that need to be divided */
    /* recopy in curv2 the subTraj that are OK                       */
    /*****************************************************************/
    for (iter_stock = curv_stock.trajList.begin(); iter_stock != curv_stock.trajList.end(); iter_stock++){
      if (iter_stock->flag_traj == 1){
        /* the error of this subTraj is too high --> need to be divided */
        iter_temp_divis->point_depart = iter_stock->point_depart;
        for (unsigned int k = 0; k < iter_stock->traj.size(); k++) {
          curv_donne = iter_stock->traj[k];
          iter_temp_divis->traj.push_back(curv_donne);
        }

        iter_temp_divis ++;
        flag_sum ++;
      }
      else if (iter_stock->flag_traj == 0){
        nb_seg_total ++;
        starting_point_each_seg = iter_stock->point_depart;
        for (unsigned int k = starting_point_each_seg; k < (iter_stock->traj.size() + starting_point_each_seg); k++) {
          if((int)k == starting_point_each_seg){
            kc_subTraj.t = k * tic;
	    for(int h=0; h<_nbAxis; h++) {
	      kc_subTraj.kc[h].x = iter_stock->traj[k-starting_point_each_seg].Pos[h];
	    }
            //kc_subTraj.kc[0].x = iter_stock->traj[k-starting_point_each_seg].Pos[0];
            //kc_subTraj.kc[1].x = iter_stock->traj[k-starting_point_each_seg].Pos[1];
            //kc_subTraj.kc[2].x = iter_stock->traj[k-starting_point_each_seg].Pos[2];
            _vec_kinpoint.push_back(kc_subTraj);
          }

	  curv2.traj[k] = iter_stock->traj[k-starting_point_each_seg];
	  curv2.traj[k].t = k * tic;
        } /* end for copy value in curv2 */
      } /* end else if (iter_stock->flag_traj == 0) -->  err < errMax*/

    }

  }while(flag_sum != 0);

  maxProfile(curv2.traj, &max_jerk, &max_acc, &max_vel);

//   for (unsigned int i = 0; i < _result.size(); i++){
//      printf("xavvvv index seg %f  time %f\n",(double)(_result[i].premier_point), (double)(_result[i].Time[0]));
// 
//   }


  
  Calcul_Error(_curve.begin()->traj, curv2.traj, &_curve.begin()->errorMax, _err_traj, &errMax_pos_traj);
    
  //printf("errMax_pos_traj %f\n",errMax_pos_traj);
  curv2.errorMaxVal = errMax_pos_traj;
  curv2.discPoint = _curve.front().discPoint;
  curv2.errorMax = _curve.front().errorMax;
  _curve.push_back(curv2);

//   printf("tutu\n");
// SM_TRAJ smTraj;
// 
//   _result[0].IC.resize(_result[0].Time.size());
// 
//   for(unsigned int i=0; i<_result[0].Time.size(); i++) {
//     _result[0].IC[i].x = _curve.begin()->traj[0].Pos[i];
//     _result[0].IC[i].a = 0;
//     _result[0].IC[i].v = 0;
//   }
// 
//  
// 
//    smTraj.importFromSM_OUTPUT(36, _result);
// 
//    smTraj.print();
//   std::vector< SM_COND> cond;
//   smTraj.getMotionCond(smTraj.getDuration(), cond);
// printf("tutu1\n");
//   for(unsigned int i=0; i<cond.size(); i++) {
//     double err = fabs(cond[i].x -  (_curve.back()).traj[ (_curve.back()).traj.size()-1].Pos[i] );
//     if( err > 0.001) {
//       printf("ERROR final pose on axis %d , err= %f \n",i, err);
//     }
//   }
// 
//  printf("tutu2\n");
  return;
  
}

void Sm_Approx::loadSvgFile(std::string str)
{
  Sm_Curve curv;
  this->_curve.push_back(curv);
}


void Sm_Approx::computeHausdorff(){
  _flag_haus_actif = 1;

  //   std::vector<double> dis_a_tracer1;
  //   std::vector<double> dis_a_tracer2;

  double sup1 = 0.0;
  double sup2 = 0.0;
  double dis_hausdorff = 0.0;
  double w = 0.0;

  // f1 pour calculer la distance la plus longue entre courbe1 et courbe2
  for (int i=0; i< (int)_curve.front().traj.size(); i++){
    std::vector<double> dis1;
    for (int j=0; j<  (int)_curve.back().traj.size(); j++){
      w = sqrt(pow((_curve.front().traj[i].Pos[0]-_curve.back().traj[j].Pos[0]),2) +  pow((_curve.front().traj[i].Pos[1]-_curve.back().traj[j].Pos[1]),2) + pow((_curve.front().traj[i].Pos[2]-_curve.back().traj[j].Pos[2]),2));
      dis1.push_back (w);
    }
    double inf1 = dis1[0];
    for (int k=0; k<(int)_curve.back().traj.size(); k++){
      if (dis1[k]<inf1) {inf1 = dis1[k];}
    }
    _err_haus1.push_back(inf1);
  }
  sup1 = _err_haus1[0];
  for (int m=0; m<(int)_curve.front().traj.size(); m++){
    if (_err_haus1[m]>(sup1)) {sup1 = _err_haus1[m];}
  }

  // f2 pour calculer la distance la plus longue entre courbe2 et courbe1
  for (int i=0; i< (int)_curve.back().traj.size(); i++){
    std::vector<double> dis2;
    for (int j=0; j< (int)_curve.front().traj.size(); j++){
      w = sqrt  (pow((_curve.back().traj[i].Pos[0]-_curve.front().traj[j].Pos[0]),2) + pow((_curve.back().traj[i].Pos[1]-_curve.front().traj[j].Pos[1]),2) + pow((_curve.back().traj[i].Pos[2]-_curve.front().traj[j].Pos[2]),2));
      dis2.push_back(w);
    }
    double inf2 = dis2[0];
    for (int k=0; k<(int)_curve.front().traj.size(); k++){
      if (dis2[k]<inf2) {inf2 = dis2[k];}
    }
    _err_haus2.push_back(inf2);
  }
  sup2 = _err_haus2[0];
  for (int m=0; m<(int)_curve.back().traj.size() ; m++){
    if (_err_haus2[m]>(sup2)) {sup2 = _err_haus2[m];}
  }

  // calcul de la distance hausdorff
  dis_hausdorff = (sup1 > sup2 ? sup1 : sup2);
  LOG(INFO,  "Hausdorff distance = " << dis_hausdorff);

  return;
  
}

void Sm_Approx::resetPlanner(){
  _curve.clear();
  _fileName.clear();
  _nbCurve = 0;
  return;
}
