close all
clear

fcn = {'robfcn',@MobilePlanarRevolute,'obsfcn', @Cuboid, 'loadfcn', @Cuboid};
[rb,ob,ld] = scenario(4,20,1,'grid',5,'dof',2,fcn{:});