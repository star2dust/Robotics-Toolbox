% test polycons
close all
clear

a = rand(50,3); 
% a = h2e(SO3.rpy(rand(1,3)).T*e2h([rand(2,50);zeros(1,50)]))';
% a = [1:10;1:10;1:10]'/10;
% a = [0.3,0.3,0.3];
plot3(a(:,1),a(:,2),a(:,3),'b*'); hold on
ia = convhull_(a);
plot3(a(ia,1),a(ia,2),a(ia,3),'g-'); 
axis equal;