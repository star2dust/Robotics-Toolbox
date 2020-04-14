% test ployxpoly_ and convhull_
close all
clear

% a = rand(2,5)'; b = rand(2,5)';

% empty intersection
a = [0.9614    0.7837;
    0.6733    0.8407;
    0.4374    0.8600;
    0.4209    0.7037;
    0.9614    0.7837];
b = [0.0089    0.9960;
    0.2752    0.0335;
    0.7841    0.1704;
    0.8888    0.2834;
    0.0089    0.9960];
% on one line
% a = [1:10;1:10]'/10;
% only one point
% a = [0.3,0.3];

% show points
plot(a(:,1),a(:,2),'g*');hold on
plot(b(:,1),b(:,2),'b*');
% teset convhull_
a = a(convhull_(a),:);
b = b(convhull_(b),:);
plot(a(:,1),a(:,2),'g-');
plot(b(:,1),b(:,2),'b-');
[c,d] = polyxpoly_(a,b);
plot(c(:,1),c(:,2),'r-');
plot(d(:,1),d(:,2),'ro');