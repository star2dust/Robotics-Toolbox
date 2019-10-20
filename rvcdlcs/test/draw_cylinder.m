figure
hold all
% Plot lower surface
center = [0,0,0];
radius = 5;
NOP = 1000;
THETA=linspace(0,2*pi,NOP);
RHO=ones(1,NOP)*radius;
[X,Y] = pol2cart(THETA,RHO);
X=X+center(1);
Y=Y+center(2);
Z = center(3)*ones(1,length(X));
h = fill3(X,Y,Z,'y'); alpha(h,0.5);
% Plot upper surface
center = [0,0,1];
radius = 5;
NOP = 1000;
THETA=linspace(0,2*pi,NOP);
RHO=ones(1,NOP)*radius;
[X,Y] = pol2cart(THETA,RHO);
X=X+center(1);
Y=Y+center(2);
Z = center(3)*ones(1,length(X));
h = fill3(X,Y,Z,'y'); alpha(h,0.5);
% Plot cylinder
[X,Y,Z] = cylinder(5,100);
[TRI,v]= surf2patch(X,Y,Z); 
patch('Vertices',v,'Faces',TRI,'facecolor',[1 1 0.5],'facealpha',0.5);
view(3);axis square; grid on; title('Enclosed Cylinder')
hold off