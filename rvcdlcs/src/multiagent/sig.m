function [y,f] = sig(x,p,e)
% Calculate sig function
if nargin<3
   e = 10^-3;
end
nx = normby(x,1);
for i=1:length(nx)
    if nx(i)<=e
        f = true;
    else
        f = false;
    end
end
y = (sign(x).*abs(x).^p);
end