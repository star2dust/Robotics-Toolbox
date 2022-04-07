function [nodez, nodepoly, nodezinpoly] = polyinit(nodez,nodepoly,nodezinpoly,config,envir,robotnum,dispbaseref,dispendref)
% polyinit    Initialize a polygraph

% set nodez to all zero
znum = size(nodez,1);
polynum = length(nodepoly);
% generate nodes outside obstacles
randnode = [envir.start;envir.goal];
randnum = size(randnode,1);
for i=1:randnum
    % generate ploytope
    randpoly = polyinflate(envir.obstacle,randnode(i,:),envir);
    % check whether formation exists in a polytope
    [randz,exit] = poly2nodez(randpoly,randnode(i,:),config,envir,robotnum,dispbaseref,dispendref);
    if exit==1
        % add this location to nodelocation list
        nodez = [nodez;randz];
        nodepoly = [nodepoly,randpoly];
        nodezinpoly{polynum+i} = znum+i;
    end
end
end