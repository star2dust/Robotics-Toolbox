% https://ww2.mathworks.cn/matlabcentral/answers/113066-help-on-matlab-isosurface-3d-object-projects-to-2-d-plane

% mesh:
xx=1:20;
yy=1:20;
zz=1:20;
[X,Y,Z] = meshgrid(xx,yy,zz);
%init the test volume data test
test = zeros(size(X));
%small cube at lower
test(5:8,5:8,5:7)=50;
%large cube at higher
test(7:11,7:11,8:11)=60;
%isosurface
p= patch(isosurface(X,Y,Z,test,1));
isonormals(X,Y,Z,test,p)
set(p,'FaceColor','red','EdgeColor','none');
daspect([1 1 1])
view(3); axis([1,20,1,20,1,20])
camlight 
lighting gouraud
grid on