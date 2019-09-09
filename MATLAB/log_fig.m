close all;
clc;

mouse = xlsread('logexcel.xlsx');

ax(1) = subplot(4,1,1);
plot(mouse(1:1000,1:2));
ax(2) = subplot(4,1,2);
plot(mouse(1:1000,3:4));
ax(3) = subplot(4,1,3);
plot(mouse(1:1000,5:6));
ax(4) = subplot(4,1,4);
plot(mouse(1:1000,7:8));

linkaxes(ax,'x');

figure;
plot(mouse(1:1000,5:6));
figure;
plot(mouse(1:1000,7:8));
