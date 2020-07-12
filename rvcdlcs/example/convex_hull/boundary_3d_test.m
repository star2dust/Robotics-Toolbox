close all
clear

a = rand(50,3); 
% a = h2e(SO3.rpy(rand(1,3)).T*e2h([rand(2,50);zeros(1,50)]))';
% a = [1:10;1:10;1:10]'/10;
% a = [0.3,0.3,0.3];
plot3(a(:,1),a(:,2),a(:,3),'b*'); hold on
axis equal;

s = 0.5;


kb = boundary_(a,s);
plot3(a(kb,1),a(kb,2),a(kb,3),'r-'); 
% trisurf(kb,a(:,1),a(:,2),a(:,3),'Facecolor','r','FaceAlpha',0.1)

c = a(interior(a,s),:);
plot3(c(:,1),c(:,2),c(:,3),'mo'); 
