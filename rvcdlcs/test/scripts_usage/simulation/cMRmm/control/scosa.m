close all
clear

% both concave when f1,f2>0
% convex when f1,f2<0
[s,a] = meshgrid(-1:.1:1,-pi:.1:pi);
f1 = s.*cos(a);
f2 = s.*sin(a);
surf(s,a,f1);
figure
surf(s,a,f2);

