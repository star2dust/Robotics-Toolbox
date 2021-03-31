function [nodez,nodepoly,nodezinpoly] = polyrand(nodez,nodepoly,nodezinpoly,randnum,config,envir,robotnum,dispbaseref,dispendref)
% Generate random nodes and add it into the node set of a polygraph

% set nodez to all zero
znum = size(nodez,1);
polynum = length(nodepoly);
% add nan between obstacles
obstpoly = envir.obstacle{1}';
for i=2:length(envir.obstacle)
    obstpoly = [obstpoly,NaN(2,1),envir.obstacle{i}'];
end
% discretization
ngrid = 100;
xgrid = (envir.ub(1)-envir.lb(1))/ngrid;
ygrid = (envir.ub(2)-envir.lb(2))/ngrid;
[xloc,yloc] = meshgrid(envir.lb(1):xgrid:envir.ub(1),...
    envir.lb(2):ygrid:envir.ub(2));
% choose node outside any existing polytope and obstacle
xlocn = []; ylocn = [];
for i=1:length(xloc(:));
    % if this node is not inside any obstacle
    if ~inpolygon(xloc(i),yloc(i),obstpoly(1,:),obstpoly(2,:))
        % chech if inside a existing polytope
        if ~innodepoly([xloc(i),yloc(i)],nodepoly)
            xlocn = [xlocn,xloc(i)];
            ylocn = [ylocn,yloc(i)];
        end
    end
end
% randomly choose n node
indn = 1:length(xlocn);
rdn = indn(randperm(numel(indn),randnum));
% generate random location around these nodes
ctr = 1;
loop = 0;
while (ctr<=randnum)
    loop = loop+1;
    % maxium loop for one random node
    if loop>10
       ctr = ctr+1; 
    end
    % generate random location around these nodes
    randlocation = [rand*2*xgrid+xlocn(rdn(ctr))-xgrid;
        rand*2*ygrid+ylocn(rdn(ctr))-ygrid]';
    % generate ploytope
    randpoly = polyinflate(envir.obstacle,randlocation,envir);
    % check whether formation exists in a polytope
    [randz,exit] = poly2nodez(randpoly,randlocation,config,envir,robotnum,dispbaseref,dispendref);
    if exit==1
        % add this location to nodelocation list
        nodez = [nodez;randz];
        nodepoly = [nodepoly,randpoly];
        nodezinpoly{polynum+ctr} = znum+ctr;
        %%%%%
        %                 hold on
        %                 plot(randlocation(1),randlocation(2),'go');
        %                 plot(randz(1),randz(2),'r*');
        %                 hold off
        %%%%%
        ctr=ctr+1;
        loop = 0;
    end
end
end

function flag = innodepoly(randlocation,nodepoly)
polynum = length(nodepoly);
count = 0;
for i=1:polynum
    if nodepoly(i).A*randlocation'-nodepoly(i).b<=0
        count = count+1;
    end
end
if count==0
    flag = false;
else
    flag = true;
end
end