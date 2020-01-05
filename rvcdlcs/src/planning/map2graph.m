% transfer map matrix to graph
function [bigraph,bilocation] = map2graph(map)
nx = size(map,1); ny = size(map,2); nz = size(map,3); n = nx*ny*nz;
bigraph = zeros(n);
for i=1:n-1
    for j=i+1:n
        if map(i)==0&&map(j)==0
            [ix,iy,iz] = ind2sub(size(map),i);
            [jx,jy,jz] = ind2sub(size(map),j);
            % three cases: face, edge or vertex contact
            if norm([ix,iy,iz]-[jx,jy,jz])==1||norm([ix,iy,iz]-[jx,jy,jz])==sqrt(2)||norm([ix,iy,iz]-[jx,jy,jz])==sqrt(3)
                if sum(sum(map([ix,jx],[iy,jy],[iz,jz])))==0
                    bigraph(i,j) = 1;
                    bigraph(j,i) = 1;
                end
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