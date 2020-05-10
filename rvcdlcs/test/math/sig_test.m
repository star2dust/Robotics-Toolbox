close all
clear

x = [0:0.01:1;-1:0.01:0];
a = 0.2;
y = sign(x).*abs(x).^a;
figure
plot(x(1,:),y(1,:));
figure
plot(x(2,:),y(2,:));