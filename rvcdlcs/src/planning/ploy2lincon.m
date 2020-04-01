function [A, b] = ploy2lincon(poly)
% generate a linear constraint A*x<=b from the vertices of a polygon

ind = convhull(poly(:,1),poly(:,2));
A = []; b = [];
for i=1:length(ind)-1
    p1 = poly(ind(i),:)'; p2 = poly(ind(i+1),:)';
    % wedge2(p2-p1)*(x-p1)>=0
    A = [A;-wedge2(p2-p1)];
    b = [b;-wedge2(p2-p1)*p1];
end
Ab = unique([A,b],'rows');
A = Ab(:,1:end-1);
b = Ab(:,end);
end