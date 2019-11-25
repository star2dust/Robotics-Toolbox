close all
clear

% intersection
pm = [1.05;1.05]; % minimum
pm = [1.20;1.20]; % maximum
syms x y real
f1 = (cos(-x-y)+cos(-y)+1)/2-pm(1);
f2 = (sin(-x-y)+sin(-y)+1)/2-pm(2);
re = solve(f1,f2);
for i=1:length(re.x)
    if eval(re.x(i))<=0&&eval(re.y(i))<=0
        pt0 = eval([re.x(i),re.y(i)]);
    end
end
% constraints
A(1,:) = [-1,-1]; b(1,:) = -pi/2;
A(2,:) = [0,1]; b(2,:) = 0;
k1 = -1/(1+sin(pt0(2))/sin(sum(pt0)));
b1 = pt0(2)-k1*pt0(1);
A(3,:) = -[k1,-1]; b(3,:) = -b1;
k2 = -1/(1+cos(pt0(2))/cos(sum(pt0)));
b2 = pt0(2)-k2*pt0(1);
A(4,:) = [k2,-1]; b(4,:) = b2;
% projection point
px = [-0.6;-0.2];
plot(px(1),px(2),'bo');
% proj test
[pj,ps] = cvxproj(px,A,b);
hold on
axis equal
patch(ps(1,:),ps(2,:),'y');
plot(pj(1),pj(2),'ro');