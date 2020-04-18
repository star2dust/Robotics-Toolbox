function [Vc, Vx] = polyxpoly_(V1,V2)
% polyxpoly_ - Intersection points for lines or polygon edges (2D)
%
%     This function returns the intersection convex hull Vc and points Vx
%     of two polylines in a planar, Cartesian system, with vertices defined
%     by V1 and V2.
%
%     [Vc, Vx] = polyxpoly_(V1,V2)


% Intersection points for lines or polygon edges
% same as MATLAB's polyxpoly
if size(V1,2)==2&&size(V2,2)==2
%     plot(V1(:,1),V1(:,2),'c:');
%     plot(V2(:,1),V2(:,2),'y:');
    xyV1 = num2cell(V1*1000/1000,1);
    xyV2 = num2cell(V2*1000/1000,1);
    [xVx, yVx] = polyxpoly(xyV1{:},xyV2{:});
    Vx = [xVx(:), yVx(:)];
%     plot(Vx(:,1),Vx(:,2),'ro')
    % Intersection convex set of polygon 1 and polygon 2
    if iscolinear(V1)||iscolinear(V2)
        % lines
        Vc = Vx;
    else
        % polygons
        V12 = [V1;V2];
        IN = inpolygon(V12(:,1), V12(:,2), xyV1{:});
        V12(IN == 0, :) = []; % discard points not in polygon 1
        IN = inpolygon(V12(:,1), V12(:,2), xyV2{:});
        V12(IN == 0, :) = []; % discard points not in polygon 2
        Vc = [V12;Vx];
        Vc = Vc(convhull_(Vc),:);
%         plot(Vc(:,1),Vc(:,2),'r-')
    end
else
    error('unknown size');
end