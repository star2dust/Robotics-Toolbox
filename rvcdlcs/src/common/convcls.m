function [V,I] = convcls(V)
% generate the convex hull by extending all the vertices (2D)

Vc = sum(V)/size(V,1);
[A, b] = convlincon(V-Vc);
for i=1:size(V,1)
    if ~ismember(i,convhull(V))
        V(i,:)=lsqlin(eye(2),(V(i,:)-Vc)'*1000,A,b);
    else
        V(i,:) = V(i,:);
    end
end
th= cart2pol(V(:,1)-Vc(1),V(:,2)-Vc(2));
[~,I] = sort(th);
I = [I(:);I(1)];
V = V(I,:);
end