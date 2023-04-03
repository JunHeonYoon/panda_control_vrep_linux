clc; close all; clear all;

file_name = 'data/hw_6.txt';
delimiterIn = ' ';
headerlineIn = 0;

data = importdata(file_name, delimiterIn, headerlineIn);
data = data(1:8000, :);
t = length(data);
time = 0.001*(1:t);

figure("Name", "Position of Obstacle and End-effector")
[X,Y,Z] = sphere;
r = 0.05;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;
surf(X2+0.15,Y2-0.012,Z2+0.65); hold on
plot3(data(:, 1), data(:, 2), data(:, 3), "r", "LineWidth",3)
plot3(0.3, -0.012, 0.52, "ko", "MarkerSize",10, "MarkerFaceColor","black"); hold off
legend(["Obst", "Traj", "Goal"])
grid on; xlabel("X"); ylabel("Y"); zlabel("Z")
axis([0 0.4 -0.2 0.2 0.4 0.8])

figure("Name","End-effector Position");
subplot(3, 1, 1)
plot(time, data(:, 4), "k--", "LineWidth", 3); hold on
plot(time, data(:, 1), "r"); hold off
title("X" ); xlabel("Time [sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(3, 1, 2)
plot(time, data(:, 5), "k--", "LineWidth", 3); hold on
plot(time, data(:, 2), "r"); hold off
title("Y"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])
subplot(3, 1, 3)
plot(time, data(:, 6), "k--", "LineWidth", 3); hold on
plot(time, data(:, 3), "r"); hold off
title("Z"); xlabel("Time[sec]"); ylabel("Position [m]"); grid on
legend(["Desired", "Operated"])