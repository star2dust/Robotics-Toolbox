% update mobile mR manipulator figure
function h = MmRupdate(h,ge,th,xi,l)
import SE3.*
lm = l(1); la = l(2:end-1); le = l(end); 
% reference config for each joint
for i=1:length(ge)
    g_sl0{i} = transl(0,0,0);
end
la0 = [0;la(1:end-1)];
for i=1:length(th)   
    g_sl0{i+3} = transl(sum(la0(1:i))+la(i)/2,0,0);
end
% reference config
g_st0 = transl(sum(la),0,0);
% structure
rod = [[0,0,0]',[1,0,0]']-[0.5,0,0]';
cub = ([0,1.5,1.5,0,0;0,0,1,1,0;0,0,0,0,0]-[.75,.5,0]')/1.5;
grp = ([0,-2,-2,0;0,0,1,1;0,0,0,0]-[-2,.5,0]')/2;
link_dat{1} = cub*lm; link_dat{length(l)} = grp*le;
for i=1:length(th)
link_dat{i+1} = rod*la(1);
end
% plot robot given th Fe 
mFm = zeros(3,1);
q = [mFm;th];
mTe = g_st0;
for k=length(q):-1:1
    mTe = expm(hatwedge(xi{k}).*q(k))*mTe;
end
eTm = invg(mTe);
Te = rt2tr(rotz(ge(3)),[ge(1:2);0]);
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
    if j>=length(Fm)
        link{j-length(Fm)+1} = g_sl{j}*e2h(link_dat{j-length(Fm)+1});
    end
    if j==length(q)
        link{j-length(Fm)+2} = g_st*e2h(link_dat{j-length(Fm)+2});
    end
end
% update
for i=1:length(link)
    set(h(i), 'XData', link{i}(1,:), 'YData', link{i}(2,:), 'ZData', link{i}(3,:));
end
end