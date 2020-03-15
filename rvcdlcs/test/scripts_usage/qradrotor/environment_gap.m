% generate obstacle environment with grid cell
% - edge: edge length of grid
% - smap: size of map matrix
% - pct: percentage of obstacles (from 0 to 1) (0: no obs, 1: full obs)
% - gap: the minimun number of grids between obstacles
function [obs, qobs, map, mapg] = environment_gap(edge,smap,pct,gap)
nrow = smap(1); ncol = smap(2); nlay = smap(3);
map = rand(smap)<pct;
if gap>1
%     if nlay==1
%         mapg = zeros(gap*nrow,gap*ncol,nlay);
%         for i=1:nrow*ncol*nlay
%             [ix,iy,iz] = ind2sub(smap,i);
%             for j = (iz-1)*gap+1:iz*gap
%                 if map(i)==1
%                     mapg((ix-1)*gap+1:ix*gap,(iy-1)*gap+1:iy*gap,iz) = (mapg((ix-1)*gap+1:ix*gap,(iy-1)*gap+1:iy*gap,iz)+(rand(gap)<0.5))>0;
%                 end
%             end
%         end
%     else
        mapg = zeros(gap*nrow,gap*ncol,gap*nlay);
        for i=1:nrow*ncol*nlay
            [ix,iy,iz] = ind2sub(smap,i);
            if map(i)==1
                for j = (iz-1)*gap+1:iz*gap
                    mapg((ix-1)*gap+1:ix*gap,(iy-1)*gap+1:iy*gap,j) = rand(gap)<0.5;
                end
            end
        end
%     end
else
    mapg = map;
end
lx = edge(1); ly = edge(2); lz = edge(3);
qobs = []; smapg = size(mapg);
nrow = smapg(1); ncol = smapg(2); nlay = smapg(3);
% if nlay~=1
%     nlay = smapg(3);
% end
for i=1:nrow*ncol*nlay
    if mapg(i)==1       
       if ~(i==1)&&~(i==nrow*ncol*nlay) % start and goal
           loc = ind2loc(smapg,i,[lx,ly,lz]);
           qobs = [qobs;loc,zeros(1,3)];
       else
           mapg(i)=0;
       end
    end
end
for i=1:size(qobs,1)
    obs(i) = Cuboid(edge,'name',['obs' num2str(i)]);
end
end