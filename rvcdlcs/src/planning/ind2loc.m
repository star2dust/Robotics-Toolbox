function loc = ind2loc(siz,ind,lgrid)
% get real coordinate from map index 
% - siz: size of map matrix
% - ind: map index (from 1 to smap(1)*smap(2)*smap(3))
% - lgrid: edge length of grid
% - loc: real coordinate of grid center at first quadrant (same size as lgrid)
% |---------y  
% |              =>   map matrix
% |x

[irow,icol,ilay] = ind2sub(siz,ind); 
if length(lgrid)==3
    sub = [irow,icol,ilay];
else
    sub = [irow,icol];
end
loc = sub2loc(sub,lgrid);
% lx = lgrid(1); ly = lgrid(2); lz = lgrid(3);
% x = (irow-1)*lx+lx/2;
% y = (icol-1)*ly+ly/2;
% z = (ilay-1)*lz+lz/2;
% loc = [x,y,z];
end