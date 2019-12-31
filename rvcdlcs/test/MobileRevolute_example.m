close all
clear


ed = [3,2,1];
c1 = Cuboid(ed,'name','cub1');
c1.plot([0,0,0,0,0,1],'facecolor','y','frame');hold on
c2 = Cuboid(ed/2,'name','cub2');

m = 2; l = ones(m,1);
xi = mRtwist(l);
dh = poe2dh(xi);
mR = SerialLink(dh,'name','mR', 'plotopt', {'notiles', 'noshading', 'noshadow', 'floorlevel', -1});
mR.plot([1,2]);
c1.plot([0,0,0,0,0,2]);
mR.plot([2,1]);