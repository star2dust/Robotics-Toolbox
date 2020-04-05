function [A, b] = polycons(V)
% generate a linear constraint A*x<=b inside a polygon

% counterclockwise
Vconv = V(convhull(V),:); 
vec = diff(Vconv);
for i=1:size(vec,1)
    A(i,:) = -skew2(vec(i,:));
    b(i,:) = -skew2(vec(i,:))*Vconv(i,:)';
end
% Remove duplicates
Ab = unique([A,b],'rows');
A = Ab(:,1:end-1);
b = Ab(:,end);
% ind = convhull(vertices(:,1),vertices(:,2));
% A = []; b = [];
% for i=1:length(ind)-1
%     p1 = vertices(ind(i),:)'; p2 = vertices(ind(i+1),:)';
%     % wedge2(p2-p1)*(x-p1)>=0
%     A = [A;-wedge2(p2-p1)];
%     b = [b;-wedge2(p2-p1)*p1];
% end
% Ab = unique([A,b],'rows');
% A = Ab(:,1:end-1);
% b = Ab(:,end);
end