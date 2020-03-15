function pt_proj = th_proj(pt_test,pt_intsec)
ps = pt_intsec;
pt = pt_test;
% syms x y real
% constraints vector
% v1 = [cos(-pi/4);sin(-pi/4)];
% v2 = [-1;0];
% v3 = [1;-1/(1+sin(ps(2))/sin(sum(ps)))];
% v4 = [1;-1/(1+cos(ps(2))/cos(sum(ps)))];
% constraints
% c1 = -x-y-pi/2;
% c2 = y;
k1 = -1/(1+sin(ps(2))/sin(sum(ps)));
b1 = ps(2)-k1*ps(1);
k2 = -1/(1+cos(ps(2))/cos(sum(ps)));
b2 = ps(2)-k2*ps(1);
% c3 = y-k1*x-b1;
% c4 = k2*x+b2-y;
% intersection
% rs0 = [solve(c3,c4),solve(c1,c4);];
% rs = [eval([rs0.x]),-pi/2,-b1/k1;
%       eval([rs0.y]),0,0;];
rs = [-pi/2,-b1/k1,-(b2-b1)/(k2-k1),-(2*b2 + pi)/(2*(k2 + 1));
    0,0,(k2*b1-b2*k1)/(k2-k1),(2*b2 - pi*k2)/(2*(k2 + 1))];
% condition 1: inside
if check_inside(pt,ps)
    pj = pt;
else
    % condition 2: edge
%     v0 = pt-[x;y];
%     re0 = [solve(v0'*v1,c1),solve(v0'*v3,c3),solve(v0'*v4,c4)];
%     re = [eval([re0.x]),pt(1);
%         eval([re0.y]),0];
    re = [pt(1)/2 - pt(2)/2 - pi/4,pt(1),(pt(1) - b1*k1 + k1*pt(2))/(k1^2 + 1),(pt(1) - b2*k2 + k2*pt(2))/(k2^2 + 1);
        pt(2)/2 - pt(1)/2 - pi/4,0,(pt(2)*k1^2 + pt(1)*k1 + b1)/(k1^2 + 1),(pt(2)*k2^2 + pt(1)*k2 + b2)/(k2^2 + 1)];
%     plot(re(1,:),re(2,:),'r.');
    i = 0;
    while i<length(re(1,:))
        i=i+1;
        if ~check_inside(re(:,i),ps)
            re(:,i)=[];
            i=i-1;
        end
    end
    % condition 3: intersection
    res = [re,rs];
    dis = 1000;
    for i=1:length(res)
        if norm(pt-res(:,i))<dis
            pj = res(:,i);
            dis = norm(pt-res(:,i));
        end
    end 
end
pt_proj = pj;
% convas
% figure
% patch(rs(1,:),rs(2,:),'y');
% axis equal
% hold on
% plot(pt(1),pt(2),'bo');
% plot(pj(1),pj(2),'ro');
end