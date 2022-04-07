function h = polyanimate(h,A,b)
% Animate a polytope
if ~isempty(A)
    V = lcon2vert(A, b);
    k = convhull(V(:,1), V(:,2));
    set(h,'xdata',V(k,1),'ydata', V(k,2));
end
end