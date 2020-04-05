function [V,I] = closure2(V)
% generate the closure of vertices (2D)

Vc = sum(V,1)/size(V,1);
[th, rho]= cart2pol(V(:,1)-Vc(1),V(:,2)-Vc(2));
[~,I] = sort(th);
I = [I(:);I(1)];
V = V(I,:);
end