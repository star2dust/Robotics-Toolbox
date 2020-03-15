function [pose, ind] = map2cub(map,edge)
% generate obstacle environment with Cuboid/Cuboid2 grid cell from map matrix
% - map: map matrix with elements 0 or 1
% - edge: edge length of grid
% - pose: pose of Cuboid (Nx6)/ Cuboid2 (Nx3)

siz = size(map); 
if length(siz)==2
    siz(3) = 1;
end
pose = []; ind = [];
nrow = siz(1); ncol = siz(2); nlay = siz(3);
for i=1:nrow*ncol*nlay
    if map(i)==1       
       if ~(i==1)&&~(i==nrow*ncol*nlay) 
           obs_loc = ind2loc(siz,i,edge);
           if length(edge)==3
               pose = [pose;obs_loc,zeros(1,3)];
           else
               pose = [pose;obs_loc,0];
           end
           ind = [ind,i];
       else % start and goal are zeros
           map(i)=0;
       end
    end
end
end