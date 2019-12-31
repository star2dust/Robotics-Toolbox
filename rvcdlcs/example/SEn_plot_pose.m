close all
clear

% test SE2,SE3 plot and transfer to pose
a = SE2([1,2,3]);
b = SE3.qrpy([2,3,0,1,2,3]);
% SE3
hb = b.plot('color','g','arrow','length',0.5,'style','-.');hold on
% SE2
ha = a.plot('color','r','length',0.5,'style','--'); 

a.q
b.toqrpy

c = SE3.qeul([2,3,0,1,2,3]);
c.toqeul

