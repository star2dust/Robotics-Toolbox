function Vh = hatwedge(V)
% transfer local representation V to twist Vh
v = V(1:3);
w = V(4:6);
Vh = [skew(w),v;
    zeros(1,4)];
end