close all
clear

syms th1 th2 th3 real
J = [-sin(th1)-sin(th1+th2)-sin(th1+th2+th3), -sin(th1+th2)-sin(th1+th2+th3),-sin(th1+th2+th3);
    cos(th1)+cos(th1+th2)+cos(th1+th2+th3), cos(th1+th2)+cos(th1+th2+th3),cos(th1+th2+th3)];
mu = simplify(sqrt(det(J*J')));
mu

[th2,th3] = meshgrid(-1.6:.1:1.6,-1.6:.1:1.6);
c1 = max(th3,-th3-pi/2);
c2 = max(th3+th2,-th3-th2-pi/2);
c = max(c1,c2);
z = cos(th2) - cos(2*th2 + th3) - cos(2*th2)/2 - cos(2*th3) - cos(2*th2 + 2*th3) - cos(th2 + 2*th3) + cos(th3) + 5/2;
surf(th2,th3,z);
xlabel('th2');ylabel('th3');
figure
zmax = max(max(z));
za = zmax*abs(sin(th2+th3));
surf(th2,th3,za);


syms th1 th2 th3 th4 real
J = [-sin(th1)-sin(th1+th2)-sin(th1+th2+th3)-sin(th1+th2+th3+th4), -sin(th1+th2)-sin(th1+th2+th3)-sin(th1+th2+th3+th4),-sin(th1+th2+th3)-sin(th1+th2+th3+th4),-sin(th1+th2+th3+th4);
    cos(th1)+cos(th1+th2)+cos(th1+th2+th3)+cos(th1+th2+th3+th4), cos(th1+th2)+cos(th1+th2+th3)+cos(th1+th2+th3+th4),cos(th1+th2+th3)+cos(th1+th2+th3+th4),+cos(th1+th2+th3+th4)];
mu = simplify(sqrt(det(J*J')));
mu

[th2,th3,th4] = meshgrid(-1.6:.1:1.6,-1.6:.1:1.6,-1.6:.1:1.6);
c1 = max(th3,-th3-pi/2);
c2 = max(th3+th2,-th3-th2-pi/2);
c = max(c1,c2);
z = cos(th2 - th4) - cos(th2 + 2*th3) - cos(2*th2 + th3) - 2*cos(th3 + 2*th4) - 2*cos(2*th3 + th4) - cos(2*th2)/2 - cos(2*th3) - (3*cos(2*th4))/2 - (3*cos(2*th2 + 2*th3 + 2*th4))/2 - cos(2*th2 + 2*th3) - 2*cos(2*th3 + 2*th4) - cos(th2 + th3 + 2*th4) - 2*cos(th2 + 2*th3 + th4) - cos(2*th2 + th3 + th4) + cos(th2 + th3) + cos(th2 + th4) + cos(th3 + th4) + 3*cos(th2) + 3*cos(th3) + 4*cos(th4) - 2*cos(th2 + 2*th3 + 2*th4) - 2*cos(2*th2 + 2*th3 + th4) + 15/2;
zmax = max(max(max(z)));

% prepare the colorbar limits
maxcolor = zmax;   % define the maximum of the data function
mincolor = 0;   % define the minimum of the data function
% colormap bone

figure
slice(th2,th3,th4,z, 0, 0, 0);
xlabel('th2');ylabel('th3');zlabel('th4');
shading interp
alpha(0.75) 
caxis([mincolor maxcolor])
colorbar
figure
za = maxcolor*abs(sin(th2+th3+th4));
slice(th2,th3,th4,za, 0, 0, 0);
xlabel('th2');ylabel('th3');zlabel('th4');
shading interp
alpha(0.75) 
caxis([mincolor maxcolor])
colorbar


