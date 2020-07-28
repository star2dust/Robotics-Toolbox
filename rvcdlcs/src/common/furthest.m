function [I,V] = furthest(V)
% furthest - Find the furthest two vertices in a set of vertices 
% - V = [X,Y] or V = [X,Y,Z], where X and Y are column vectors.
% - I = [i,j] is the index of furthest two vertices in V

dist = 0;
for i=1:size(V)-1
    for j=i:size(V)
        if norm(V(i,:)-V(j,:))>dist
           dist = norm(V(i,:)-V(j,:));
           I = [i,j]';
        end
    end
end
V = V(I,:);
end