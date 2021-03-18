function [g,ng,png_pt,hg] = circbarrier(x,t,c,r,dc,dr)

if nargin<5
   dc = zeros(size(c));
   dr = zeros(size(r));
end
x = x(:);
c = c(:);
r = r(:);
dc = dc(:);
dr = dr(:);
% set rho parameters (想离边界远可设a2=1e-3，想允许一定的误差可设k1=1)
a1 = 1; a2 = 1e-3; k1 = 1;
rho = @(t) a1*exp(a2*t); drho = @(t) a2*rho(t);
f = (x-c)'*(x-c)-r^2;
g = -log(k1-rho(t)*f)/rho(t);
ng = 2*(x-c)*(1./(k1-rho(t)*f));
png_pt = -2*dc*(1./(k1-rho(t)*f))+...
    2*(x-c)*(drho(t)*f./(k1-rho(t)*f).^2)+...
    2*(x-c)*(rho(t)*(-2*(x-c)'*dc-2*r*dr)./(k1-rho(t)*f).^2);
hg = 2./(k1-rho(t)*f).^2+2*(x-c)*2*(x-c)'*(rho(t)./(k1-rho(t)*f).^2);
end