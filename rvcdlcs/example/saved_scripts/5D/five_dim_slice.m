%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                5D Data Visualization                 %
%              with MATLAB Implementation              %
%                                                      %
% Author: M.Sc. Eng. Hristo Zhivomirov        04/01/13 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all
% set(gcf, 'Renderer', 'Painter')   % vector graphics 
% set(gcf, 'Renderer', 'Zbuffer')   % faster and more accurate than Painters 
set(gcf, 'Renderer', 'OpenGL')      % speed up the rendering process

% form the axes
x = -1:0.1:1;   % first dimension indipendent variable
y = -1:0.1:1;   % second dimension indipendent variable
z = -1:0.1:1;   % third dimension indipendent variable

% form the 3D grid
[X, Y, Z] = meshgrid(x, y, z);

% prepare the colorbar limits
maxcolor = 4;   % define the maximum of the data function
mincolor = 0;   % define the minimum of the data function
colormap bone

% animation
for A = -1:0.01:1   % fourth dimension indipendent variable

% write the equation that describes the fifth dimension
data = 1/abs(X.^2 + Y.^2 + Z.^2 + A.^2);

% plot the data
figure(1)
slice(X, Y, Z, data, 0, 0, 0);
shading interp
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14)
xlabel('X')                             
ylabel('Y')                             
zlabel('Z')                            
title(['F(X, Y, Z, A) = 1/abs((X^2 + Y^2 + Z^2 + A^2)), for A = ' num2str(A)])
alpha(0.75) 
caxis([mincolor maxcolor])
colorbar

drawnow

end