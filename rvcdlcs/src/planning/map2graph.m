% transfer map matrix to graph
function [bigraph,bilocation] = map2graph(map)
nx = size(map,1); ny = size(map,2); nz = size(map,3); n = nx*ny*nz;
bigraph = zeros(n);
for i=1:n-1
    for j=i+1:n
        if map(i)==0&&map(j)==0
            [ix,iy,iz] = ind2sub(size(map),i);
            [jx,jy,jz] = ind2sub(size(map),j);
            % any vertice pair inside an obstacle free district has an edge
            ijx = min(ix,jx):max(ix,jx);
            ijy = min(iy,jy):max(iy,jy);
            ijz = min(iz,jz):max(iz,jz);
            if sum(sum(map(ijx,ijy,ijz)))==0
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