function h = polyplot(A,b)
% Plot a polytope
if ~isempty(A)
    V = lcon2vert(A, b);
    k = convhull(V(:,1), V(:,2));
    h = plot(V(k,1), V(k,2), 'ro-', 'LineWidth', 2);
end
end