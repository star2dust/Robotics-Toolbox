close all
clear

% intersection
scale = 1;
pm = [0.97;0.68]; % minimum
% pm = [1.09;0.8]; % maximum
syms x y real
f1 = (cos(-x-y)+cos(-y)+1)/2-pm(1);
f2 = (sin(-x-y)+sin(-y))/2-pm(2);
r = solve(f1,f2);
for i=1:length(r.x)
   if eval(r.x(i))<=0&&eval(r.y(i))<=0
       pt0 = eval([r.x(i),r.y(i)]);
   end
end
% constraints th2 th3
[th2,th3] = meshgrid(-1.6:.001:1.6,-1.6:.001:1.6);
% cth1 = max(-th2-th3-pi/2,th2+th3);
% cth2 = max(-th3-pi/2,th3);
% cth = max(cth1,cth2);
c1 = ((cos(-th2-th3)+cos(-th3)+1)/2-pm(1))*scale;
c2 = ((sin(-th2-th3)+sin(-th3))/2-pm(2))*scale;
c3 = max(-th2-th3-pi/2,th2+th3);
c4 = max(-th3-pi/2,th3);
cth = max(c1,max(c2,max(c3,c4)));
% convexity
% x0 = -1.6:.01:1.6;
% k1 = -1/(1+sin(pt0(2))/sin(sum(pt0)));
% b1 = pt0(2)-k1*pt0(1);
% k2 = -1/(1+cos(pt0(2))/cos(sum(pt0)));
% b2 = pt0(2)-k2*pt0(1);
% cp1 = th3-k1*th2-b1;
% cp2 = k2*th2+b2-th3;
% cp = max(cp1,cp2);
cp=-100;
% final constraints
c = max(cp,cth);
ind = find(c<0);
figure
plot(th2(ind),th3(ind),'.y')
figure
plot((cos(-th2(ind)-th3(ind))+cos(-th3(ind))+1)/2,(sin(-th2(ind)-th3(ind))+sin(-th3(ind)))/2,'.y')


% x0 = -1.6:.01:0;
% pt0 = -[0.29;0.64];
% k1 = -1/(1+sin(pt0(2))/sin(sum(pt0)));
% b1 = pt0(2)-k1*pt0(1);
% y1 = k1*x0+b1;
% plot(x0,y1,'r')
% 
% 
% k2 = -1/(1+cos(pt0(2))/cos(sum(pt0)));
% b2 = pt0(2)-k2*pt0(1);
% y2 = k2*x0+b2;
% plot(x0,y2,'b')

