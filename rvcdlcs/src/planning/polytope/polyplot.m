function h = polyplot(A,b,c)
% Plot a polytope
if nargin<3
   c = 'ro-';
end
if ~isempty(A)
    V = lcon2vert(A, b);
    k = convhull(V(:,1), V(:,2));
    h = plot(V(k,1), V(k,2), c, 'LineWidth', 2);
end
end