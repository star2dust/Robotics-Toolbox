% https://ww2.mathworks.cn/matlabcentral/answers/339366-regrid-isosurfaces-to-2d-arrays

% Housekeeping
clear all; close all
% Initialiaze figure
figure('units','normalized','outerposition',[0 0 1 0.5])
% Create 3D arrays
dx = 100; dy =  50; dz = 25;
nx = 200; ny = 100; nz = 100;
xs = linspace(0,dx,nx);
ys = linspace(0,dy,ny);
zs = linspace(0,dz,nz);
[X,Y,Z] = meshgrid( xs, ys, zs);
my_array = sin(0.3.*pi+0.4.*pi.*X./dx).*sin(0.3.*pi+0.4.*pi.*Y./dy).*(Z./dz);
% Find isosurface
p = isosurface(X,Y,Z,my_array,0.1);
x = p.vertices(:,1) ;
y = p.vertices(:,2) ;
z = p.vertices(:,3) ;
tri = p.faces ;
% Plot isosurface in 3D
subplot(1,2,1)
trisurf(tri,x,y,z) ;
shading flat
colorbar;
% Re-grid to 2D arrays
[xi,yi] = meshgrid(xs, ys);
zi = griddata(x,y,z,xi,yi);
% Plot isosurface in 2D
subplot(1,2,2)
pcolor(xi,yi,zi);
shading flat
colorbar;