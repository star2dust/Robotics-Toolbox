function [I,V] = interior(V,s)
% interior - Generate the relative interior of a set of vertices 
% - V = [X,Y] or V = [X,Y,Z], where X and Y are column vectors.
% - I : the index of vertices in V
Iv = ones(1,size(V,1));
if nargin<2
    Ib = boundary_(V);
else
    Ib = boundary_(V,s);
end
V(Ib,:) = [];
Iv(Ib) = 0;
I = find(Iv==1);
end