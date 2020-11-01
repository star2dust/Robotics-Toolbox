function [A,b] = polycons(x)
% polycons - Generate a linear constraint A*x<=b for 2D convex hull
% - x = [X,Y], where X and Y are column vectors.
% - A,b : the linear constraint A*x<=b

if size(x,2)==2
    if isempty(x)||size(x,1)==1
        A = []; 
        b = []; 
    else
        % counterclockwise
        xsort = x(convhull_(x),:);
        vec = diff(xsort);
        for i=1:size(vec,1)
            A(i,:) = -skew_(vec(i,:));
            b(i,:) = -skew_(vec(i,:))*xsort(i,:)';
        end
        % Remove duplicates
        Ab = unique([A,b],'rows');
        A = Ab(:,1:end-1);
        b = Ab(:,end);
    end
else
    error('unknown size');
end