% mR manipulator model

function mR = mdl_mR(l)
if nargin<1
   m = 2; l = ones(m,1);
end
xi = mRtwist(l);
dh = poe2dh(xi);
mR = SerialLink(dh,'name','mR', 'plotopt', {'notiles', 'noshading', 'noshadow', 'floorlevel', -1});
mR.plot([1,2]);
end