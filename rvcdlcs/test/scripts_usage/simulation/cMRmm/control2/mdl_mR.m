% mR manipulator model

function mR = mdl_mR(m)
if nargin<1
   m = 2; 
end
l = ones(m,1);
xi = mRtwist(l);
dh = poe2dh(xi);
mR = SerialLink(dh(1:end-1),'name','mR');
end