% test polycons
close all
clear

a = rand(5,2); 
a = a(convhull_(a),:);
plot(a(:,1),a(:,2),'g-'); hold on
axis([0 2 0 2]);

c = rand(1,2)*2;
[Aa,ba] = polycons(a);
P = ind2sub_([10 10],1:100)/10;
P = P(sum(Aa*P'<=ba)==length(ba),:);
plot(P(:,1),P(:,2),'bo')

ca = lsqlin(eye(2),c',Aa,ba)';
plot([c(1),ca(1)],[c(2), ca(2)],'ro-');