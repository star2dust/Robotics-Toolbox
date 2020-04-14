close all
clear

load('Stest.mat','S');
plot(S(:,1),S(:,2),'b*');
k = convhull_(S);
