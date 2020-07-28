function [A,b,Aeq,beq] = polycons(V)
% polycons - Generate a linear constraint A*x<=b for 2D convex hull
% - V = [X,Y] or V = [X,Y,Z], where X and Y are column vectors.
% - A,b : the linear constraint A*x<=b

if size(V,2)==2
    if isempty(V)
        A = []; Aeq = [];
        b = []; beq = [];
    elseif size(V,1)==1
        A = []; Aeq = eye(2);
        b = []; beq = V';
    else
        % counterclockwise
        Vconv = V(convhull_(V),:);
        vec = diff(Vconv);
        for i=1:size(vec,1)
            A(i,:) = -skew_(vec(i,:));
            b(i,:) = -skew_(vec(i,:))*Vconv(i,:)';
        end
        % Remove duplicates
        Ab = unique([A,b],'rows');
        A = Ab(:,1:end-1);
        b = Ab(:,end);
        Aeq = [];
        beq = [];
    end
else
    error('unknown size');
end