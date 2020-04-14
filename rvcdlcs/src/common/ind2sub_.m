function sub = ind2sub_(siz,ind)
% ind2sub_ - Subscripts from linear index
% 
%     This function returns the matrices SUB = [I,J] containing the equivalent row
%     and column subscripts corresponding to each linear index in the matrix IND for a
%     matrix of size siz.
% 
%     SUB = ind2sub_(siz,IND)

ind = ind(:);
if length(siz)==2
    [ix,iy] = ind2sub(siz,ind);
    sub = [ix,iy];
else
    [ix,iy,iz] = ind2sub(siz,ind);
    sub = [ix,iy,iz];
end
end