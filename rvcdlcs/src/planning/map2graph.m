function [bigraph,bilocation] = map2graph(map)
nx = size(map,1); ny = size(map,2); nz = size(map,3); n = nx*ny*nz;
bigraph = zeros(n);
for i=1:n-1
    for j=i+1:n
        if map(i)==0&&map(j)==0
            [ix,iy,iz] = ind2sub(size(map),i);
            [jx,jy,jz] = ind2sub(size(map),j);
            if norm([ix,iy,iz]-[jx,jy,jz])==1
                bigraph(i,j) = 1;
                bigraph(j,i) = 1;
            end
        end
    end
end
bilocation = zeros(n,3);
for i=1:n
   [ix,iy,iz] = ind2sub(size(map),i);
   bilocation(i,:) = [ix,iy,iz];
end
end