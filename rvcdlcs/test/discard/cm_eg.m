close all
clear

% frames
% - r: reference
% - c: centroid
% - m: mobile base
% symbols
% - g: SE2/SE3
% - p: translation (2x1)
% - phi: rotation (1x1)
% - q = [p',phi]: translation and rotation (1x3)
% relations 
% - g_r: g_r relative to inertia frame
% - g_r2c: g_c relative to r frame
% - g_r2em: g_em relative to r frame

nrob = 4; nlink = 3; radius = .5;
cm = CooperativeManipulation(nrob,nlink,radius,'name','cm');

g_0 = SE2; 
% psi = 0; scale = 1; 
% g_r = SE2([2.5,2.5,0]);
% g_r2c = SE2; g_c = g_r*g_r2c;
% th_r = cm.formationIkine(g_r2c,psi,scale);
% p_r2m_hat = cm.formationTransl(g_r2c,scale);
% p_m_hat = h2e(g_r.T*e2h(p_r2m_hat));
% 
% figure
% cm.plot(g_r*g_r2c,psi,th_r(:,2:end),'workspace',[0 10 0 10 0 10]/2-0.1,'frame','framelength',[.25 .5],'framethick',[1 2]); hold on
% h = plot(p_m_hat(1,[1:end,1]),p_m_hat(2,[1:end,1]));
% g_0.plot('color','m','frame','I');
% g_c.plot('color','g','frame','C');
% g_r.plot('color','g','frame','R');
% view(2)


psi = pi/8*2; scale = 1-0.15;
g_r = SE2([2.5,2.5,0]+[0,0,1]/2); 
g_r2c = SE2([1,1]/2)*SE2(rot2(-pi/8)); g_c = g_r*g_r2c; 
th_r = cm.formationIkine(g_r2c,psi,scale);
p_r2m_hat = cm.formationTransl(g_r2c,scale);
p_m_hat = h2e(g_r.T*e2h(p_r2m_hat));

figure
cm.plot(g_r*g_r2c,psi,th_r(:,2:end),'workspace',[0 10 0 10 0 10]/2-0.1,'frame','framelength',[.25 .5],'framethick',[1 2]); hold on
h = plot(p_m_hat(1,[1:end,1]),p_m_hat(2,[1:end,1]));
g_0.plot('color','m','frame','I');
g_c.plot('color','g','frame','C');
g_r.plot('color','g','frame','R');
view(2)

