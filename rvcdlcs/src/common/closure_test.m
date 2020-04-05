close all
clear

a = rand(500,2); 
plot(a(:,1),a(:,2),'b*'); hold on
axis([0 1 0 1]);

a = a(closure(a),:);
plot(a(:,1),a(:,2),'b-'); 
