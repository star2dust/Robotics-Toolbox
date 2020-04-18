function f = iscolinear(V)
% iscolinear - Test if colinear (2D) or coplanar (3D)
%     iscolinear(V) is true (1) if vertex set V is colinear or coplanar or empty,
%     else false (0).
if isempty(V)||size(V,1)<=size(V,2)
    f = true;
else
    Vc = sum(V)/size(V,1);
    V = V-Vc;
    f = rank(V)<size(V,2);
end
end