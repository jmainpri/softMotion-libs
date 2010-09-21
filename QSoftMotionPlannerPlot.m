 % to modify manually
 % open file.fig
 % saveas(gcf, 'file', 'epsc2');


 function QSoftMotionPlannerPlot(fileName)
 french = 0;
%fileName = ['droite'];
withJerk = 0;
fileCurve = ['SmCurves_',fileName, '.dat'];
fileDiscr = ['SmDiscr_',fileName, '.dat'];
fileMaxError = ['SmMaxError_',fileName, '.dat'];
data = load(fileCurve);
dataDiscr = load(fileDiscr);
dataMaxError = load(fileMaxError);



  fileName = ['SmCurves_',fileName,'_'];

 index = data(:,1);
 time = data(:,2);

 ideal.p.x = data(:,3);
 ideal.p.y = data(:,4);
 ideal.p.z = data(:,5);

 ideal.v.x = data(:,6);
 ideal.v.y = data(:,7);
 ideal.v.z = data(:,8);

 ideal.a.x = data(:,9);
 ideal.a.y = data(:,10);
 ideal.a.z = data(:,11);

 approx.p.x = data(:,12);
 approx.p.y = data(:,13);
 approx.p.z = data(:,14);

 approx.v.x = data(:,15);
 approx.v.y = data(:,16);
 approx.v.z = data(:,17);

 approx.a.x = data(:,18);
 approx.a.y = data(:,19);
 approx.a.z = data(:,20);

 errorTraj =  data(:,21);
 errorVit = data(:,22);
 errorHauss1 =  data(:,23);
 errorHauss2 =  data(:,24);


 motionLaw.a = data(:,25);
 motionLaw.v = data(:,26);
 motionLaw.x = data(:,27);

 idealMotionLaw.a = data(:,28);
 idealMotionLaw.v = data(:,29);
 idealMotionLaw.x = data(:,30);

 approxMotionLaw.a = data(:,31);
 approxMotionLaw.v = data(:,32);
 approxMotionLaw.x = data(:,33);
 figNum = 1;

if(max(ideal.p.z)-min(ideal.p.z) ==0)
    is2D = 1;
else
   is2D = 0;
end


 disp 'ploting the ideal 3D curve ...'
 figure(figNum)

  if(is2D==0)
  plot3(ideal.p.x, ideal.p.y, ideal.p.z);
  else
        plot(ideal.p.x, ideal.p.y);
  end

 grid on
 xlabel('Px (m)');
 ylabel('Py (m)');
 if(is2D==0)
 zlabel('Pz (m)');
 end
 if(french)
     legend('Trajectoire ideale') ;
 else
     legend('Ideal trajectory') ;
 end
 hold off
 saveName = (['fig',fileName, '3dIdealCurve']);
 saveas(gcf, saveName, 'epsc2');
 saveas(gcf, saveName, 'fig');
 close
 disp '   ... OK'


 figNum = figNum +1;
 disp 'ploting the results 3D curve ...'
 figure(figNum)
 if(is2D==0)
 plot3(ideal.p.x, ideal.p.y, ideal.p.z, '--');
  hold on
 grid on
 plot3(approx.p.x,approx.p.y, approx.p.z, 'r');
 plot3(dataDiscr(:,3),dataDiscr(:,4),dataDiscr(:,5),'s','MarkerFaceColor','g', 'MarkerSize',5);
 plot3(dataMaxError(:,3),dataMaxError(:,4),dataMaxError(:,5),'d','MarkerFaceColor','r', 'MarkerSize',10);
 xlabel('Px (m)');
 ylabel('Py (m)');
 zlabel('Pz (m)');
 else
      plot(ideal.p.x, ideal.p.y,  '--');
  hold on
 grid on
 plot(approx.p.x,approx.p.y,  'r');
 plot(dataDiscr(:,3),dataDiscr(:,4),'s','MarkerFaceColor','g', 'MarkerSize',5);
 plot(dataMaxError(:,3),dataMaxError(:,4),'d','MarkerFaceColor','r', 'MarkerSize',10);
 xlabel('Px (m)');
 ylabel('Py (m)');
 end
 if(french)
     legend('Trajectoire ideale', 'Trajectoire approximee','Intervalle des sous-trajectoires de 3 segments', 'Erreur maximale') ;
 else
     legend('Ideal trajectory', 'Approximated trajectory', 'Bounds of 3-segments sub-trajectories', 'Maximal Error') ;
 end
 hold off
 saveName = (['fig',fileName, '3dResultCurve']);
 saveas(gcf, saveName, 'epsc2');
 saveas(gcf, saveName, 'fig');
 close
 disp '   ... OK'




  figNum = figNum +1;
 disp 'ploting the  motion law imposed ...'
 figure(figNum)



if(withJerk)
plot(time, computeJerk(time,motionLaw.a) ,'color','green','LineStyle',':');
end
  hold on
 grid on
 plot(time, motionLaw.a ,'color', 'blue','LineStyle','--');

  plot(time, motionLaw.v , 'red');
   plot(time, motionLaw.x,'color', 'black','LineStyle','-.');
 if(french)
 title('Loi de mouvement impose');
 xlabel('temps (s)');
 if(withJerk)
  legend('jerk (m/s^3)','acceleration (m/s^2)', 'vitesse (m/s)', 'position (m)') ;
 else
     legend('acceleration (m/s^2)', 'vitesse (m/s)', 'position (m)') ;
 end
 else
 title('Imposed motion law');
 xlabel('time (s)');
 if(withJerk)
 legend('jerk (m/s^3)','acceleration (m/s^2)', 'velocity (m/s)', 'position (m)') ;
 else
    legend('acceleration (m/s^2)', 'velocity (m/s)', 'abscissa (m)') ;
 end
 end
 hold off

 saveName = (['fig',fileName, 'MLaw']);
 saveas(gcf, saveName, 'epsc2');
 saveas(gcf, saveName, 'fig');
 close
 disp '   ... OK'



 figNum = figNum +1;
 disp 'ploting the ideal motion law  ...'
 figure(figNum)

 plot(time, idealMotionLaw.a ,'color', 'blue','LineStyle','--');
  hold on
 grid on
  plot(time, idealMotionLaw.v , 'red');
   plot(time, idealMotionLaw.x,'color', 'black','LineStyle','-.');
 if(french)
 title('Loi de mouvement ideale');
 xlabel('temps (s)');
  legend('acceleration (m/s^2)', 'vitesse (m/s)', 'position (m)') ;
 else
 title('Ideal motion law');
 xlabel('time (s)');
 legend('acceleration (m/s^2)', 'velocity (m/s)', 'abscissa (m)') ;
 end
 hold off

 saveName = (['fig',fileName, 'IdealMLaw']);
 saveas(gcf, saveName, 'epsc2');
 saveas(gcf, saveName, 'fig');
 close
 disp '   ... OK'


  figNum = figNum +1;
 disp 'ploting the approximated  motion law...'
 figure(figNum)

 plot(time, approxMotionLaw.a ,'color', 'blue','LineStyle','--');
  hold on
 grid on
  plot(time, approxMotionLaw.v , 'red');
   plot(time, approxMotionLaw.x,'color', 'black','LineStyle','-.');
 if(french)
 title('profil de mouvement approximee');
 xlabel('temps (s)');
  legend('acceleration (m/s^2)', 'vitesse (m/s)', 'position (m)') ;
 else
 title('Approximated motion law');
 xlabel('time (s)');
 legend('acceleration (m/s^2)', 'velocity (m/s)', 'abscissa (m)') ;
 end
 hold off

 saveName = (['fig',fileName, 'ApproxMProfil']);
 saveas(gcf, saveName, 'epsc2');
 saveas(gcf, saveName, 'fig');
 close
 disp '   ... OK'





 figNum = figNum +1;
 disp 'ploting the ideal axis trajectory...'
 figure(figNum)
  if(is2D==0)
 subplot(3,1,1);
  else
       subplot(2,1,1);
  end
 hold on
 grid on
 plot(time, ideal.a.x ,'color', 'blue','LineStyle','--');
 plot(time, ideal.v.x, 'red');
 plot(time, ideal.p.x,'color', 'black','LineStyle','-.');
 if(french)
 title('(a) Trajectoire ideale suivant X');
 xlabel('temps (s)');
 else
 title('(a) Ideal trajectory of the X axis');
 xlabel('time (s)');
 legend('acceleration (m/s^2)', 'velocity (m/s)', 'abscissa (m)') ;
 end
 hold off

   if(is2D==0)
 subplot(3,1,2);
  else
       subplot(2,1,2);
  end

 hold on
 grid on
 plot(time, ideal.a.y ,'color', 'blue','LineStyle','--');
 plot(time, ideal.v.y, 'red');
 plot(time, ideal.p.y,'color', 'black','LineStyle','-.');
 if(french)
 title('(b) Trajectoire ideale suivant Y');
 xlabel('temps (s)');
 else
 title('(b) Ideal trajectory of the Y axis');
 xlabel('time (s)');
%  legend('acceleration (m/s^2)', 'velocity (m/s)', 'position (m)') ;
 end
 hold off


   if(is2D==0)
     subplot(3,1,3);
     hold on
     grid on
     plot(time, ideal.a.z ,'color', 'blue','LineStyle','--');
     plot(time, ideal.v.z, 'red');
     plot(time, ideal.p.z,'color', 'black','LineStyle','-.');
     if(french)
     title('(c) Trajectoire ideale suivant Z');
     xlabel('temps (s)');
     else
     title('(c) Ideal trajectory of the Z axis');
     xlabel('time (s)');
%      legend('acceleration (m/s^2)', 'velocity (m/s)', 'position (m)') ;
     end
     hold off
   end

 saveName = (['fig',fileName, 'IdealTraj']);
 saveas(gcf, saveName, 'epsc2');
 saveas(gcf, saveName, 'fig');
 close
 disp '   ... OK'



 figNum = figNum +1;
 disp 'ploting the error trajectory of the X axis ...'
 figure(figNum)
 subplot(3,1,1);
 hold on
 grid on
 plot(time, abs(ideal.a.x - approx.a.x));
  if(french)
  xlabel('temps (s)');
 ylabel('(a) Erreur Ax (m/s^2)');
  else
 xlabel('time (s)');
 ylabel('(a) Ax error (m/s^2)');
  end
 hold off
 subplot(3,1,2);
 hold on
 grid on
 plot(time, abs(ideal.v.x- approx.v.x));
  if(french)
  xlabel('temps (s)');
 ylabel('(b) Erreur Vx (m/s)');
  else
 xlabel('time (s)');
 ylabel('(b) Vx error (m/s)');
  end
 hold off
 subplot(3,1,3);
 hold on
 grid on
 plot(time, abs(ideal.p.x-approx.p.x));
  if(french)
  xlabel('temps (s)');
  ylabel('(b) Erreur Px (m)');
  else
  xlabel('time (s)');
  ylabel('(b) Px error (m)');
  end
 hold off
 saveName = (['fig',fileName, 'ErrorTrajX']);
 saveas(gcf, saveName, 'epsc2');
 saveas(gcf, saveName, 'fig');
 close
 disp '   ... OK'


  figNum = figNum +1;
 disp 'ploting the error trajectory of the Y axis ...'
 figure(figNum)
 subplot(3,1,1);
 hold on
 grid on
 plot(time, abs(ideal.a.y - approx.a.y));
  if(french)
  xlabel('temps (s)');
 ylabel('(a) Erreur Ay (m/s^2)');
  else
 xlabel('time (s)');
 ylabel('(a) Ay error (m/s^2)');
  end
 hold off
 subplot(3,1,2);
 hold on
 grid on
 plot(time, abs(ideal.v.y- approx.v.y));
  if(french)
  xlabel('temps (s)');
 ylabel('(b) Erreur Vy (m/s)');
  else
 xlabel('time (s)');
 ylabel('(b) Vy error (m/s)');
  end
 hold off
 subplot(3,1,3);
 hold on
 grid on
 plot(time, abs(ideal.p.y-approx.p.y));
  if(french)
  xlabel('temps (s)');
  ylabel('(b) Erreur Py (m)');
  else
  xlabel('time (s)');
  ylabel('(b) Py error (m)');
  end
 hold off
 saveName = (['fig',fileName, 'ErrorTrajY']);
 saveas(gcf, saveName, 'epsc2');
 saveas(gcf, saveName, 'fig');
 close
 disp '   ... OK'



 if(is2D==0)
   figNum = figNum +1;
 disp 'ploting the error trajectory of the Z axis ...'
 figure(figNum)
 subplot(3,1,1);
 hold on
 grid on
 plot(time, abs(ideal.a.z - approx.a.z));
  if(french)
  xlabel('temps (s)');
 ylabel('(a) Erreur Az (m/s^2)');
  else
 xlabel('time (s)');
 ylabel('(a) Az error (m/s^2)');
  end
 hold off
 subplot(3,1,2);
 hold on
 grid on
 plot(time, abs(ideal.v.z- approx.v.z));
  if(french)
  xlabel('temps (s)');
 ylabel('(b) Erreur Vz (m/s)');
  else
 xlabel('time (s)');
 ylabel('(b) Vz error (m/s)');
  end
 hold off
 subplot(3,1,3);
 hold on
 grid on
 plot(time, abs(ideal.p.z-approx.p.z));
  if(french)
  xlabel('temps (s)');
  ylabel('(b) Erreur Pz (m)');
  else
  xlabel('time (s)');
  ylabel('(b) Pz error (m)');
  end
 hold off
 saveName = (['fig',fileName, 'ErrorTrajZ']);
 saveas(gcf, saveName, 'epsc2');
 saveas(gcf, saveName, 'fig');
 close
 disp '   ... OK'
 end

   figNum = figNum +1;
 disp 'ploting the trajectory error ...'
 line = [ dataMaxError(1,2) 0 ; dataMaxError(1,2)  max(errorTraj)]
 figure(figNum)
 line(:,1)
 line(:,2)
 plot(line(:,1), line(:,2), 'r', 'LineWidth', 4);
   hold on
  grid on
  plot(time, errorTraj);
if(french)
%ylabel('Erreur trajectoire (m)');
xlabel('temps (s)');

str = ['Erreur maximale ', sprintf('%f', max(errorTraj)), ' (m)'];
str
 legend(str, 'Erreur trajectoire (m)');
else
 ylabel('Trajectory error (m)');
 xlabel('time (s)');

 str = ['Maximal Error ', sprintf('%f', max(errorTraj)), ' (m)'];
str
 legend(str, 'Trajectory Error (m)');
end
 hold off
 saveName = (['fig',fileName, 'ErrorTraj']);
 saveas(gcf, saveName, 'epsc2');
 saveas(gcf, saveName, 'fig');
 close
 disp '   ... OK'




    figNum = figNum +1;
 disp 'ploting the velocity error ...'
 figure(figNum)
  hold on
  grid on
  plot(time, errorVit);
if(french)
xlabel('temps (s)');

% str = ['Erreur maximale ', sprintf('%f', max(errorTraj)), ' (m)'];
%  legend(str, 'Erreur vitesse (m/s)');
legend('Erreur vitesse (m/s)');
else
 %ylabel('velocity error (m)');
 xlabel('time (s)');
end
 hold off
 saveName = (['fig',fileName, 'ErrorVit']);
 saveas(gcf, saveName, 'epsc2');
 saveas(gcf, saveName, 'fig');
 close
 disp '   ... OK'



 end
