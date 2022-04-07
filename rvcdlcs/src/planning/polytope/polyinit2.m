function [nodez, nodepoly, nodezinpoly] = polyinit2(nodez,nodepoly,nodezinpoly,config,envir,robot_num,disp_base_max,disp_end_ref)
% polyinit    Initialize a polygraph

% set nodez to all zero
znum = size(nodez,1);
polynum = length(nodepoly);
% generate nodes outside obstacles
randnode = [envir.start,pi/4;envir.goal,pi];
randnum = size(randnode,1);
for i=1:randnum
    % generate ploytope
    randpoly = polyinflate(envir.obstacle,randnode(i,1:2),envir);
    % check whether formation exists in a polytope
    [randz,exit] = poly2nodez2(randpoly,randnode(i,:),config,envir,robot_num,disp_base_max,disp_end_ref);
    if exit==1
        % add this location to nodelocation list
        nodez = [nodez;randz];
        nodepoly = [nodepoly,randpoly];
        nodezinpoly{polynum+i} = znum+i;
    end
end
end