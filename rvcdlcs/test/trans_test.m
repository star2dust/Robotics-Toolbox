close all
clear

r = 0:0.1:1;
x = kron(r,ones(size(r)));
y = kron(ones(size(r)),r);
plot(x,y,'o')
p0 = [x;y];
th1 = -pi/6; th2 = pi/6;
T1 = [1, -sin(th1-th2);
    0, cos(th1-th2)];
T2 = [cos(th2), -sin(th2);
    sin(th2), cos(th2)];
p1 = T2*T1*p0;
plot(p1(1,:),p1(2,:),'o');
axis equal

syms th1 th2;
T1 = [1, -sin(th1-th2);
    0, cos(th1-th2)];
T2 = [cos(th2), -sin(th2);
    sin(th2), cos(th2)];
simplify((T2*T1)^-1)