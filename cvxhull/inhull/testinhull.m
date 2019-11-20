% test inhull - check whether origin is in the convex hull of a,b,c,d
close all
clear

%% test function
a = [1,0,1];
b = [0,-1,-2];
c = [-1,0,1];
d = [0,1,-2];
xyz = [a;b;c;d];
in = inhull([0,0,0],xyz);
%% visualization
face = [1,2,4;
    1,2,3;
    2,3,4;
    1,3,4];
figure
hold on
view(3)
patch('Vertices',xyz,'Faces',face,'FaceColor','y','FaceAlpha',0.5);
plot3(xyz(:,1),xyz(:,2),xyz(:,3),'bo');
plot3(0,0,0,'ro');
hold off