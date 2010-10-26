clear all
close all


fileName = ['segMotion.dat'];
data = load(fileName);
index = data(:,1);
time = data(:,2);

approx.jerk.x = data(:,3);
approx.jerk.y = data(:,4);
approx.jerk.z = data(:,5);

approx.Time.prem = data(:,6);
approx.Time.deux = data(:,7);
approx.Time.troi = data(:,8);

disp 'plotting the Jerk of Motion.....'

figure
subplot(3,1,1);
hold on
grid on
stairs(time, approx.jerk.x);
xlabel('time (s)');
ylabel('Jerk_x (m)');
title('Jerk suivant X');
hold off

subplot(3,1,2);
hold on
grid on
stairs(time, approx.jerk.y);
xlabel('time (s)');
ylabel('Jerk_y (m)');
title('Jerk suivant Y');
hold off

subplot(3,1,3);
hold on
grid on
stairs(time, approx.jerk.z);
xlabel('time (s)');
ylabel('Jerk_z (m)');
title('Jerk suivant Z');
hold off

disp 'figure OK'



