function [obs, qobs, map] = environment(edge,smap,pct)
% generate obstacle environment with grid cell
% - edge: edge length of grid
% - smap: size of map matrix
% - pct: percentage of obstacles (from 0 to 1) (0: no obs, 1: full obs)
% - obs: list of Cuboid object (1xN)
% - qobs: pose of Cuboid (Nx6)
% - map: map matrix with elements 0 or 1
map = rand(smap)<pct;
lx = edge(1); ly = edge(2); lz = edge(3);
qobs = []; nrow = smap(1); ncol = smap(2); nlay = smap(3);
for i=1:nrow*ncol*nlay
    if map(i)==1       
       if ~(i==1)&&~(i==nrow*ncol*nlay) % start and goal
           loc = ind2loc(smap,i,[lx,ly,lz]);
           qobs = [qobs;loc,zeros(1,3)];
       else
           map(i)=0;
       end
    end
end
for i=1:size(qobs,1)
    obs(i) = Cuboid(edge,'name',['obs' num2str(i)]);
end
end