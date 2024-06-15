function h = polyplot(A,b,c,x0)
% Plot a polytope
if nargin<4
    x0 = [0,0]';
    if nargin<3
        c = 'ro-';
    end
end
if ~isempty(A)
    V = lcon2vert(A, b)+x0';
    k = convhull(V(:,1), V(:,2));
    h = plot(V(k,1), V(k,2), c);
%     h = plot(V(k,1), V(k,2), c, 'LineWidth', 2);
end
end