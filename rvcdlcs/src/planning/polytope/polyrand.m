function [nodez,nodepoly,nodezinpoly,glocpoly,glocn] = polyrand(nodez,nodepoly,nodezinpoly,glocpoly,randnum,config,envir,robotnum,dispbaseref,dispendref,gloc,ngrid)
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
xloc = gloc.x; yloc = gloc.y;
xlocn = []; ylocn = [];
if length(config.vec)==2
    zloc = gloc.z;
    zlocn = [];
end
for i=1:length(xloc(:));
    % if this node is not inside any obstacle
    if ~inpolygon(xloc(i),yloc(i),obstpoly(1,:),obstpoly(2,:))
        % check if the node is inside a existing polytope
        if ~innodepoly([xloc(i),yloc(i)],nodepoly,-1)
            % check if the node has been searched
            if ~inpolygon(xloc(i),yloc(i),glocpoly(1,:),glocpoly(2,:))
                xlocn = [xlocn,xloc(i)];
                ylocn = [ylocn,yloc(i)];
            end
        end
    end
end
glocn.x = xlocn; glocn.y = ylocn;
% randomly choose n node
indn = 1:length(xlocn);
rdn = indn;
for i=1:5
    rdn = rdn(randperm(length(rdn)));
end
% grid width
ngrid = ngrid-1;
xgrid = (envir.ub(1)-envir.lb(1))/ngrid;
ygrid = (envir.ub(2)-envir.lb(2))/ngrid;
% generate random location around these nodes
indc = 1; rdc = 1; 
while (rdc<=randnum)
    % generate random location around these nodes
    randlocation = [xlocn(rdn(indc)),ylocn(rdn(indc))];
    % The random location will not be searched next time
    glocpoly = [glocpoly,NaN(2,1),randlocation(1:2)'+[[-1,-1,1,1]*xgrid;[-1,1,1,-1]*ygrid]];
    % generate ploytope
    randpoly = polyinflate(envir.obstacle,randlocation(1:2),envir);
    % check whether formation exists in a polytope
    if length(config.vec)==2
        config.vec(end) = randlocation(end);
    end
    [randz,exit] = poly2nodez(randpoly,randlocation(1:2),config,envir,robotnum,dispbaseref,dispendref);
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