close all;
clc;

mouse = xlsread('logexcel.xlsx');

plot(mouse(:,1:2),'DisplayName','mouse(:,2:3)');
figure;
plot(mouse(:,3:4),'DisplayName','mouse(:,4:5)');
figure;
plot(mouse(:,9),'DisplayName','mouse(:,7:8)');
figure;
plot(mouse(:,10),'DisplayName','mouse(:,9:10)');

