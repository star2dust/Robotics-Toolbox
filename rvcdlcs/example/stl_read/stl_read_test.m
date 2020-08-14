close all
clear

[F,V] = stlread('youbot1.stl');
h = patch('faces',F,'vertices',V,'facecolor','y','edgealpha',0.2);