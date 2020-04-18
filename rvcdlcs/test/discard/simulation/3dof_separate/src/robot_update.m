function h = robot_update(h,Fe,th,xi,l)
l0 = l(1); l1 = l(2); l2 = l(3); l3 = l(4); l4 = l(5); 
% reference config for each joint
g_sl0{1} = transl(0,0,0);
g_sl0{2} = transl(0,0,0);
g_sl0{3} = transl(0,0,0);
g_sl0{4} = transl(l1/2,0,0);
g_sl0{5} = transl(l1+l2/2,0,0);
g_sl0{6} = transl(l1+l2+l3/2,0,0);
% reference config
g_st0 = transl(l1+l2+l3,0,0);
% structure
rod = [[0,0,0]',[1,0,0]']-[0.5,0,0]';
cub = [0,1.5,1.5,0,0;0,0,1,1,0;0,0,0,0,0]-[.75,.5,0]';
grp = [0,-2,-2,0;0,0,1,1;0,0,0,0]-[-2,.5,0]';
link_dat{1} = cub*l0; link_dat{2} = rod*l1; link_dat{3} = rod*l2; link_dat{4} = rod*l3; link_dat{5} = grp*l4;
% plot robot given th Fe 
mFm = zeros(3,1);
q = [mFm;th];
mTe = g_st0;
for k=length(q):-1:1
    mTe = expm(hatwedge(xi{k}).*q(k))*mTe;
end
eTm = invg(mTe);
Te = rt2tr(rotz(Fe(3)),[Fe(1:2);0]);
Tm = Te*eTm;
[Rm,tm] = tr2rt(Tm);
phim = vex(logm(Rm));
Fm = [tm(1:2);phim(3)];
% calculate again
q = [Fm;th];
g_st = g_st0;
for j=1:length(q)
    g_sl{j} = g_sl0{j};
    for k=j:-1:1
        g_sl{j} =  expm(hatwedge(xi{k}).*q(k))*g_sl{j};
        if j==length(q)
            g_st = expm(hatwedge(xi{k}).*q(k))*g_st;
        end
    end
    if j>=3
        link{j-2} = g_sl{j}*e2h(link_dat{j-2});
    end
    if j==length(q)
        link{j-1} = g_st*e2h(link_dat{j-1});
    end
end
% update
for i=1:length(link)
    set(h(i), 'XData', link{i}(1,:), 'YData', link{i}(2,:), 'ZData', link{i}(3,:));
end
end