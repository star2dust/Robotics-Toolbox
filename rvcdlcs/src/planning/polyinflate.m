function region = polyinflate(obstacle_vts,path_vts,range)
% Inflate polytopes from path points
import iris.inflate_region
ptnum = size(path_vts,1);
A_bounds = [-1,0;0,-1;1,0;0,1];
b_bounds = [-range.lb,range.ub]';
region = struct;
for i=1:length(obstacle_vts)
   obstacle_pts{i} =  obstacle_vts{i}';
end
path_pts = path_vts';
for i=1:ptnum
    [A,b,C,d,results] = inflate_region(obstacle_pts, A_bounds, b_bounds, path_pts(:,i));
    region(i).A = A;
    region(i).b = b;
    region(i).C = C;
    region(i).d = d;
end
end
