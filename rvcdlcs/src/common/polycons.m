function [A, b] = polycons(V)
% polycons - Generate a linear constraint A*x<=b for 2D convex hull
% - V = [X,Y] or V = [X,Y,Z], where X and Y are column vectors.
% - A,b : the linear constraint A*x<=b

if size(V,2)==2
    % counterclockwise
    Vconv = V(convhull(V),:);
    vec = diff(Vconv);
    for i=1:size(vec,1)
        A(i,:) = -skew_(vec(i,:));
        b(i,:) = -skew_(vec(i,:))*Vconv(i,:)';
    end
    % Remove duplicates
    Ab = unique([A,b],'rows');
    A = Ab(:,1:end-1);
    b = Ab(:,end);
else
    error('unknown size');
end