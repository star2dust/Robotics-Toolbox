function I = furthest(V)

dist = 0;
for i=1:size(V)-1
    for j=i:size(V)
        if norm(V(i,:)-V(j,:))>dist
           dist = norm(V(i,:)-V(j,:));
           I = [i,j];
        end
    end
end
end