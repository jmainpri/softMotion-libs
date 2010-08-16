 % to modify manually
 % open file.fig
 % saveas(gcf, 'file', 'epsc2');


 
 french = 1;

 data = X3dCurve;
  load 3dCurve.dat
  fileName = ['3dCurve'];

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
 errorHauss1 =  data(:,22);
 errorHauss2 =  data(:,23);
 
 figNum = 1;
 disp 'ploting the 3D curve ...'
 figure(figNum)
 hold on
 grid on
 plot3(ideal.p.x, ideal.p.y, ideal.p.z, '--');
 plot3(approx.p.x,approx.p.y, approx.p.z, 'r');
 xlabel('Px (m)');
 ylabel('Py (m)');
 zlabel('Pz (m)');
 hold off
 saveName = (['fig',fileName, '3dCurve']); 
 saveas(gcf, saveName, 'epsc2');    
 saveas(gcf, saveName, 'fig');  
 close
 disp '   ... OK'
 
 
 figNum = figNum +1;
 disp 'ploting the ideal trajectory...' 
 figure(figNum)
 subplot(3,1,1);
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
 %legend('acceleration (m/s^2)', 'velocity (m/s)', 'position (m)') ;
 end
 hold off
 
 
 subplot(3,1,2);
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
 %legend('acceleration (m/s^2)', 'velocity (m/s)', 'position (m)') ;
 end
 hold off
 
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
 %legend('acceleration (m/s^2)', 'velocity (m/s)', 'position (m)') ;
 end
 hold off
 
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
 
 figNum = figNum +1;
 disp 'ploting the trajectory error ...'
 figure(figNum)
 hold on
 grid on
 plot(time, errorTraj);
if(french) 
ylabel('Erreur trajectoire (m)');
xlabel('temps (s)');
else
 ylabel('Trajectory error (m)');
 xlabel('time (s)');
end
 hold off
 saveName = (['fig',fileName, 'ErrorTraj']); 
 saveas(gcf, saveName, 'epsc2');    
 saveas(gcf, saveName, 'fig');  
 close
 disp '   ... OK'
 
 
 


 
 
 
 
 
 