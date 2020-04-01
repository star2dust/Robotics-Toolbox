close all
clear

r = -1:0.1:1;
x = kron(r,ones(size(r)));
y = kron(ones(size(r)),r);
pxy0 = [x;y];
p0 = [0;0];
th1 = pi/6; th2 = pi/4;
% syms th1 th2
p1 = [cos(th1);sin(th1)];
p2 = [cos(th2);sin(th2)];
plot(x,y,'o')
hold on
plot([p1(1) p0(1) p2(1)], [p1(2) p0(2) p2(2)],'r-');


T = [wedge2(p1);-wedge2(p2)];
pxy0(T*pxy0<0)=nan;
figure
plot(pxy0(1,:),pxy0(2,:),'o');axis([-1 1 -1 1])
hold on
plot([p1(1) p0(1) p2(1)], [p1(2) p0(2) p2(2)],'r-');


