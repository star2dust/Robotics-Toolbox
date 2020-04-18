function [ T ] = se3Exp( xi )
%se3Exp Exponential mapping from se3 to SE3
%
%   T = se3Exp(xi) is a homogeneous transformation exponentially mapped
%   from xi
%   xi:     6x1 twist vector

DELTA=10^(-12);

n1=norm(xi(1:3));
n2=norm(xi(4:6));
if abs(n1)<DELTA % pure translation
    if abs(n2)<DELTA 
        T=eye(4);
    else
        T=se3Translation(xi(4:6)/n2,n2);
    end
else %
    T=se3Rotation(xi(1:3)/n1,xi(4:6)/n1,n1);
end

end

