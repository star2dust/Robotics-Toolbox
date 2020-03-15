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
% constraints vector
v1 = [cos(-pi/4);sin(-pi/4)];
v2 = [-1;0];
v3 = [1;-1/(1+sin(pt0(2))/sin(sum(pt0)))];
v4 = [1;-1/(1+cos(pt0(2))/cos(sum(pt0)))];
% constraints
c1 = -x-y-pi/2;
c2 = y;
k1 = -1/(1+sin(pt0(2))/sin(sum(pt0)));
b1 = pt0(2)-k1*pt0(1);
k2 = -1/(1+cos(pt0(2))/cos(sum(pt0)));
b2 = pt0(2)-k2*pt0(1);
c3 = y-k1*x-b1;
c4 = k2*x+b2-y;
% convas
figure
% intersection
rs0 = [solve(c3,c4),solve(c1,c4);];
rs = [eval([rs0.x]),-pi/2,-b1/k1;
      eval([rs0.y]),0,0;];
patch(rs(1,:),rs(2,:),'y');
axis equal
hold on
% projection point test
pj0 = [-0.6;-0.2];
pj0 = [-1.2;-0.3];
pj0 = [-1.5;0.3];
plot(pj0(1),pj0(2),'bo');
% condition 1: inside
th2 = pj0(1);
th3 = pj0(2);
cth1 = max(-th2-th3-pi/2,th2+th3);
cth2 = max(-th3-pi/2,th3);
cth = max(cth1,cth2);
k1 = -1/(1+sin(pt0(2))/sin(sum(pt0)));
b1 = pt0(2)-k1*pt0(1);
k2 = -1/(1+cos(pt0(2))/cos(sum(pt0)));
b2 = pt0(2)-k2*pt0(1);
cp1 = th3-k1*th2-b1;
cp2 = k2*th2+b2-th3;
cp = max(cp1,cp2);
% final constraints
c = max(cp,cth);
if c<=0
    plot(th2,th3,'ro')
else
    % condition 2: edge
    v0 = pj0-[x;y];
    re0 = [solve(v0'*v1,c1),solve(v0'*v3,c3),solve(v0'*v4,c4)];
    re = [eval([re0.x]),pj0(1);
        eval([re0.y]),0];
    plot(re(1,:),re(2,:),'r.');
    i = 0;
    while i<length(re(1,:))
        i=i+1;
        th2 = re(1,i);
        th3 = re(2,i);
        % constraints
        cth1 = max(-th2-th3-pi/2,th2+th3);
        cth2 = max(-th3-pi/2,th3);
        cth = max(cth1,cth2);
        k1 = -1/(1+sin(pt0(2))/sin(sum(pt0)));
        b1 = pt0(2)-k1*pt0(1);
        k2 = -1/(1+cos(pt0(2))/cos(sum(pt0)));
        b2 = pt0(2)-k2*pt0(1);
        cp1 = th3-k1*th2-b1;
        cp2 = k2*th2+b2-th3;
        cp = max(cp1,cp2);
        % final constraints
        c = max(cp,cth);
        if c>=0.000001
            re(:,i)=[];
            i=i-1;
        end
    end
    % condition 3: intersection
    res = [re,rs];
    dis = 1000;
    for i=1:length(res)
        if norm(pj0-res(:,i))<dis
            pj = res(:,i);
            dis = norm(pj0-res(:,i));
        end
    end
    plot(pj(1),pj(2),'ro')
end


