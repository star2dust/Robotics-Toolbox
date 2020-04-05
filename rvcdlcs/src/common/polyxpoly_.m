function [Vp, Vx] = polyxpoly_(V1,V2)

% Intersection points for lines or polygon edges
% same as MATLAB's polyxpoly
xyV1 = num2cell(V1,1);
xyV2 = num2cell(V2,1);
[xVx, yVx] = polyxpoly(xyV1{:},xyV2{:});
Vx = [xVx(:), yVx(:)];
% Intersection convex set of polygon 1 and polygon 2
if isempty(Vx)||size(unique(Vx,'rows'),1)==1
    % lines
    Vp = Vx; 
else
    % polygons
    Vp = [V1;V2;Vx];
    IN = inpolygon(Vp(:,1), Vp(:,2), xyV1{:});
    Vp(IN == 0, :) = []; % discard points not in polygon 1
    IN = inpolygon(Vp(:,1), Vp(:,2), xyV2{:});
    Vp(IN == 0, :) = []; % discard points not in polygon 2
    Vp = Vp(convhull(Vp),:);
end