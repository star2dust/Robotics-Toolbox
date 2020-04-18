function [obs, qobs, map] = environment(siz,tile,pct)
% generate obstacle environment with grid cell
% - edge: edge length of grid
% - smap: size of map matrix
% - pct: percentage of obstacles (from 0 to 1) (0: no obs, 1: full obs)
% - obs: list of Cuboid object (1xN)
% - qobs: pose of Cuboid (Nx6)
% - map: map matrix with elements 0 or 1
import Map.*
map = rand(siz)<pct;
qobs = []; nrow = siz(1); ncol = siz(2); nlay = siz(3);
for i=1:nrow*ncol*nlay
    if map(i)==1       
       if ~(i==1)&&~(i==nrow*ncol*nlay) % start and goal
           loc = ind2loc(siz,tile,i);
           qobs = [qobs;loc,zeros(1,3)];
       else
           map(i)=0;
       end
    end
end
for i=1:size(qobs,1)
    obs(i) = Cuboid(tile*ones(1,3),'name',['obs' num2str(i)]);
end
end