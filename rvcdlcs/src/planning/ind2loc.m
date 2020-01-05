function loc = ind2loc(smap,ind,lgrid)
[irow,icol,ilay] = ind2sub(smap,ind); irow = -irow;
lx = lgrid(1); ly = lgrid(2); lz = lgrid(3);
x = (-irow-1)*lx+lx/2;
y = (icol-1)*ly+ly/2;
z = (ilay-1)*lz+lz/2;
loc = [x,y,z];
end