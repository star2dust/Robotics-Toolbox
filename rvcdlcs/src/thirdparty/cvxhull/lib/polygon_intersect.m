function [ints_x, ints_y] = polygon_intersect(poly1_x, poly1_y, poly2_x, poly2_y)
% 求两个个凸多边形的交集 
% https://ridiqulous.com/find-the-intersection-of-convex-hull-using-matlab/
% poly1_x,poly1_y 分别为第一个多边形的各个顶点的x,y坐标，均为列向量
% poly2_x,poly2_y 分别为第二个多边形的各个顶点的x,y坐标，均为列向量
% ********************************************************** %
% Let S be the set of vertices from both polygons.
% For each edge e1 in polygon 1
%   For each edge e2 in polygon 2
%     If e1 intersects with e2
%       1. Add the intersection point to S
% Remove all vertices in S that are outside polygon 1 or 2
% ********************************************************** %
S(:,1) = [poly1_x; poly2_x]; % 将两个多边形的坐标存入 S 中，顺序无所谓
S(:,2) = [poly1_y; poly2_y];
num = size(poly1_x, 1) + size(poly2_x, 1) + 1;
for i = 1:size(poly1_x, 1) - 1
    for j =1:size(poly2_x, 1) - 1
        X1 = [poly1_x(i); poly1_x(i+1)];
        Y1 = [poly1_y(i); poly1_y(i+1)];
        X2 = [poly2_x(j); poly2_x(j+1)];
        Y2 = [poly2_y(j); poly2_y(j+1)];
        [intspoint_x, intspoint_y] = polyxpoly(X1, Y1, X2, Y2); % 求两条线段交点的x,y坐标
        if ~isempty(intspoint_x) % 若两条线段无交点则跳至下一组线段，若有交点则将交点的x,y坐标存至S中
            S(num, 1) = intspoint_x;
            S(num, 2) = intspoint_y;
            num = num + 1; % 存入 S 后往下递推一行
        end
    end
end
IN = inpolygon(S(:,1), S(:,2), poly1_x, poly1_y);
S(IN == 0, :) = []; % 剔除掉不位于多边形 A 中的顶点坐标
IN = inpolygon(S(:,1), S(:,2), poly2_x, poly2_y);
S(IN == 0, :) = []; % 剔除掉不位于多边形 B 中的顶点坐标
S = unique(S,'rows');
if iscolinear(S)
    % 判断是否为同一点或全部共线
    ints_x = S(:, 1); % 得到交集多边形的各个顶点坐标
    ints_y = S(:, 2);
else
    % 如果非全部共线也非同一点，则取凸包
    X = S(:, 1);
    Y = S(:, 2);
    k = convhull(X, Y);
    ints_x = X(k);
    ints_y = Y(k);
end
% plot(poly1_x, poly1_y, 'r', poly2_x, poly2_y, 'b', ints_x, ints_y, 'k')