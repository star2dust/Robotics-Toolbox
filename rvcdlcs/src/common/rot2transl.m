function [x,dx] = rot2transl(th,z,dth,dz)

if nargin<3
    dth = zeros(size(th));
    dz = zeros(size(z));
elseif nargin<4
    dz = zeros(size(z));
end
z = z(:);
dz = dz(:);
x = rot2(th)*z;
dx = dth*rot2(pi/2)*rot2(th)*z+rot2(th)*dz;
end