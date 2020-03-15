close all
clear

%% cpm_proj
ur = [2;2];
dl = [-1;-1];
% proj point test
pt = [3;4];
pj = cpm_proj(pt,ur,dl);

%% epc_proj
cen = [1/2;0];
r = 0.02;
% proj point test
pt = [0.0686;
    5.3331];
pj = epc_proj(pt,cen,r);


%% th_proj
% intersection
pm = [1.05,1.20;
    1.05,1.20];% [minimum,maximum]
syms x y real
for i=1:length(pm)
    f1(i) = (cos(-x-y)+cos(-y)+1)/2-pm(1,i);
    f2(i) = (sin(-x-y)+sin(-y))/2-pm(2,i);
    re(i) = solve(f1(i),f2(i));
    for j=1:length(re(i).x)
        if eval(re(i).x(j))<=0&&eval(re(i).y(j))<=0
            ps(:,i) = eval([re(i).x(j);re(i).y(j)]);
        end
    end
end
% ps = [-1.3592,  -0.2838;
%    -0.1058,   -0.6435;];
% projection point test
pt = [-1.6;-0.2];
% proj
pj = th_proj(pt,ps(:,1));




