close all
clear

a = rand(500,2); 
% a = [1:10;1:10]'/10;
% a = [0.3,0.3];
plot(a(:,1),a(:,2),'b*'); hold on
axis equal;

s = 0.5;

% ar = a(radialboundary(a),:);
% plot(ar(:,1),ar(:,2),'g-'); 

b = a(boundary_(a,s),:);
plot(b(:,1),b(:,2),'ro-'); 

c = a(interior(a,s),:);
plot(c(:,1),c(:,2),'mo'); 
