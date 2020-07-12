%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                5D Data Visualization                 %
%              with MATLAB Implementation              %
%                                                      %
% Author: M.Sc. Eng. Hristo Zhivomirov        12/13/14 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear, clc, close all
% set(gcf, 'Renderer', 'Painter')   % vector graphics 
% set(gcf, 'Renderer', 'Zbuffer')   % faster and more accurate than Painters 
set(gcf, 'Renderer', 'OpenGL')      % speed up the rendering process

% form the axes
x = 1:1:3;  % first dimension indipendent variable
y = 1:1:3;  % second dimension indipendent variable
z = 1:1:3;  % third dimension indipendent variable

% form the 3D grid
[X, Y, Z] = meshgrid(x, y, z);

% form the user data matrix
% the data could be iported from .txt or .xls file
userdata(:, :, 1) = [1  2  3;  4  5  6;  7  8  9];      % first page
userdata(:, :, 2) = [10 11 12; 13 14 15; 16 17 18];     % second page
userdata(:, :, 3) = [19 20 21; 22 23 24; 25 26 27];     % third page

% prepare the colorbar limits
mincolor = min(userdata(:));    % find the minimum of the data function
maxcolor = max(userdata(:));    % find the maximum of the data function

% animation
for A = 0:0.01:1

% plot the data
figure(1)
scatter3(X(:), Y(:), Z(:), 400, A*userdata(:), 'filled')
set(gca, 'FontName', 'Times New Roman', 'FontSize', 14)
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Userdata = \it{f} \rm(X, Y, Z)')
caxis([mincolor maxcolor])
colorbar
view(-15, 25)

drawnow

end