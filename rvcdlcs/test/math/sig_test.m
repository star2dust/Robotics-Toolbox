close all
clear

x = -1:0.01:1;
a = 0.2;
y = sign(x).*abs(x).^a;
figure
plot(x,y);