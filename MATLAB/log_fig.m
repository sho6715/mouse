close all;
clc;

mouse = xlsread('logexcel.xlsx');

ax(1) = subplot(4,1,1);
plot(mouse(1:110,1:2));
ax(2) = subplot(4,1,2);
plot(mouse(1:110,3:4));
ax(3) = subplot(4,1,3);
plot(mouse(1:110,5:6));
ax(4) = subplot(4,1,4);
plot(mouse(1:110,7:8));

linkaxes(ax,'x');

