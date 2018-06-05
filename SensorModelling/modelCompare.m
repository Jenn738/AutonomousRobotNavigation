% Engs 147 - Final Project
% Jennifer Jain
% Sensor Model Comparisson with experimental and actual distance values

close all;
clear all;
 
M = csvread('resultsEdit.csv');
volts = M(:,1);
actDist = M(:,2);
dist1 = M(:,3);
dist2 = M(:,4);
dist3 = M(:,5);
dist4 = M(:,6);
dist5 = M(:,7);
dist6 = M(:,8);
dist7 = M(:,9);

% d = linspace(0, 150, 151);
figure(1);
plot(actDist, volts,'LineWidth',1.5);
hold on;
plot(dist1, volts);
hold on;
plot(dist2, volts);
hold on;
plot(dist3, volts);
hold on;
plot(dist4, volts);
hold on;
plot(dist5, volts);
hold on;
plot(dist6, volts);
hold on;
plot(dist7, volts);
hold on;


title('IR Sensor Model Comparison');
xlabel("distance in cm");
ylabel("IR sensor reading in volts");
grid on;
grid minor;
legend('actual', 'm1', 'm2', 'm3', 'm4', 'm5', 'm6', 'm7');

% error calculations for all models
for i = 3:9
    disp(i);
    d=actDist-M(:,i);
    SSE = norm(d,2)^2
end
figure(2);
plot(actDist, volts,'LineWidth',1.5);
hold on;
plot(dist2, volts);
hold on;
title('IR Sensor Model Comparison');
xlabel("distance in cm");
ylabel("IR sensor reading in volts");
legend('actual', 'm2');
grid on;
grid minor;

