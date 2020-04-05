function h = update_region_2d(h,region)
import iris.thirdParty.polytopes.*
for i=1:length(region)
    if iscell(region)
        regionstruct = [region{:}];
    else
        regionstruct = region;
    end
    A = regionstruct(i).A;
    b = regionstruct(i).b;
    if ~isempty(A)
        V = lcon2vert(A, b);
        k = convhull(V(:,1), V(:,2));
        set(h.cvx(i),'XData',V(k,1),'YData', V(k,2));
    end
end
end