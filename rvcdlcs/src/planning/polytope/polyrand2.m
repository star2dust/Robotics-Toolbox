function [nodez,nodepoly,nodezinpoly,ptloc_old,ptlocn] = polyrand2(nodez,nodepoly,nodezinpoly,ptloc_old,randnum,config,envir,robot_num,disp_base_max,disp_end_ref,ptloc,ngrid)
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
ptlocn = [];
for i=1:size(ptloc,1);
    % if this node is not inside any obstacle
    if ~inpolygon(ptloc(i,1),ptloc(i,2),obstpoly(1,:),obstpoly(2,:))
        % check if the node is inside a existing polytope
        if ~innodepoly(ptloc(i,1:2),nodepoly,-1)||(innodepoly(ptloc(i,1:2),nodepoly,-1)&&min(normby(ptloc(i,:)-ptloc_old,1))>1)
            % check if the node has been searched
%             if ~inpolygon(ptloc(i,1),ptloc(i,2),ptlocpoly(1,:),ptlocpoly(2,:))
                ptlocn = [ptlocn;ptloc(i,:)];
%             end
        end
    end
end
% randomly choose n node
rdn = 1:size(ptlocn,1);
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
    randlocation = ptlocn(rdn(indc),:);
    % The random location will not be searched next time
    ptloc_old = [ptloc_old;randlocation];
%     ptlocpoly = [ptlocpoly,NaN(2,1),randlocation(1:2)'+[[-1,-1,1,1]*xgrid;[-1,1,1,-1]*ygrid]];
    % generate ploytope
    randpoly = polyinflate(envir.obstacle,randlocation(1:2),envir);
    % check whether formation exists in a polytope
    [randz,exit] = poly2nodez2(randpoly,randlocation,config,envir,robot_num,disp_base_max,disp_end_ref);
    if exit==1
        % add this location to nodelocation list
        nodez = [nodez;randz];
        nodepoly = [nodepoly,randpoly];
        nodezinpoly{polynum+rdc} = znum+rdc;
        %%%%%
        hold on
%         figure
%         plot3(ptlocn(:,1),ptlocn(:,2),ptlocn(:,3),'y.')
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