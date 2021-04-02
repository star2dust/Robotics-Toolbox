function [nodez,nodepoly,nodezinpoly,locpoly,xlocn,ylocn] = polyrand(nodez,nodepoly,nodezinpoly,locpoly,randnum,config,envir,robotnum,dispbaseref,dispendref,xloc,yloc)
% Generate random nodes and add it into the node set of a polygraph

% set nodez to all zero
znum = size(nodez,1);
polynum = length(nodepoly);
% add nan between obstacles
obstpoly = envir.obstacle{1}';
for i=2:length(envir.obstacle)
    obstpoly = [obstpoly,NaN(2,1),envir.obstacle{i}'];
end
% choose node outside any existing polytope and obstacle
xlocn = []; ylocn = [];
for i=1:length(xloc(:));
    % if this node is not inside any obstacle
    if ~inpolygon(xloc(i),yloc(i),obstpoly(1,:),obstpoly(2,:))
        % check if the node is inside a existing polytope
        if ~innodepoly([xloc(i),yloc(i)],nodepoly,-1)
            % check if the node has been searched
            if isempty(locpoly)||(~isempty(locpoly)&&~inpolygon(xloc(i),yloc(i),locpoly(1,:),locpoly(2,:)))
                xlocn = [xlocn,xloc(i)];
                ylocn = [ylocn,yloc(i)];
            end
        end
    end
end
% randomly choose n node
indn = 1:length(xlocn);
rdn = indn;
for i=1:5
    rdn = rdn(randperm(length(rdn)));
end
% generate random location around these nodes
indc = 1; rdc = 1; rgrid = 1;
while (rdc<=randnum)
    % generate random location around these nodes
    % randlocation = [rand*2*xgrid+xlocn(rdn(indc))-xgrid;
    %    rand*2*ygrid+ylocn(rdn(indc))-ygrid]';
    randlocation = [xlocn(rdn(indc)),ylocn(rdn(indc))];
    % The random location will not be searched next time 
    if ~isempty(locpoly)
        locpoly = [locpoly,NaN(2,1),randlocation'+[-1,-1,1,1;-1,1,1,-1]*rgrid];
    else
        locpoly = randlocation'+[-1,-1,1,1;-1,1,1,-1]*rgrid;
    end
    % generate ploytope
    randpoly = polyinflate(envir.obstacle,randlocation,envir);
    % check whether formation exists in a polytope
    [randz,exit] = poly2nodez(randpoly,randlocation,config,envir,robotnum,dispbaseref,dispendref);
    if exit==1
        % add this location to nodelocation list
        nodez = [nodez;randz];
        nodepoly = [nodepoly,randpoly];
        nodezinpoly{polynum+rdc} = znum+rdc;
        %%%%%
        hold on
        % plot(xlocn(:),ylocn(:),'y.')
        plot(randlocation(1),randlocation(2),'go');
        plot(randz(1),randz(2),'r*');
        hold off
        pause(0.01)
        %%%%%
        rdc = rdc+1;
        indc = indc+1;
    else
        indc = indc+1;
    end
end
end

function flag = innodepoly(randlocation,nodepoly,inpolyerr)
polynum = length(nodepoly);
count = 0;
for i=1:polynum
    if nodepoly(i).A*randlocation'-nodepoly(i).b<=inpolyerr
        count = count+1;
    end
end
if count==0
    flag = false;
else
    flag = true;
end
end