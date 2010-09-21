clear all
close all


fileName = ['intervale_courbure.dat'];
data = load(fileName);
courbure = data(:,1);
interval = data(:,2);

figure
hold on
grid on
plot(courbure, interval);
xlabel('courbure');
ylabel('interval');
hold off

disp 'figure OK'