function sub = ind2sub_(siz,ind)
ind = ind(:);
if length(siz)==2
    [ix,iy] = ind2sub(siz,ind);
    sub = [ix,iy];
else
    [ix,iy,iz] = ind2sub(siz,ind);
    sub = [ix,iy,iz];
end
end