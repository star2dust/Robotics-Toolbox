close all
clear

[F,V] = stlread('omni45a.stl');
h = patch('faces',F,'vertices',V,'facecolor','y','edgealpha',0.2);