close all
clear

nrob = 4; nlink = 3; radius = .5;
cm = CooperativeManipulation(nrob,nlink,radius,'name','cm');
% r: reference, c: centroid, d: displacement
g_r = SE2([2.5,2.5,0]);
g_r2c = SE2; psi = 0; scale = 1;
[g_r2e,p_r2m_hat,th_r] = cm.gfkine(g_r2c,psi,scale);
g_c = g_r*g_r2c; g_e = g_r*g_r2e; p_m_hat = h2e(g_r.T*e2h(p_r2m_hat));

figure
cm.plot(g_c.q,g_e.q,th_r(:,2:end),'workspace',[0 10 0 10 0 10]/2,'frame'); hold on
h = plot(p_m_hat(1,[1:end,1]),p_m_hat(2,[1:end,1]));
view(-30,70)

% write video
video_on = false;
if video_on
    respath = '/home/chu/Documents/MATLAB/Toolboxes/Robotics-Toolbox/rvcdlcs/result/';
    videoname = [respath 'cooperative_manipulation_demo_v1.2'];
    writerObj = VideoWriter(videoname);
    open(writerObj);
end
for t=0:0.05:3
    if t<=1
        g_r2c = SE2(rot2(-pi/4*t)); psi = pi/4*t; scale = 1-0.3*t;
    elseif t>1&&t<=2
        g_r2c = SE2(rot2(pi/2*(t-1)-pi/4)); psi = pi/4-pi/4*(t-1); scale = 1-0.3+0.6*(t-1);
    else
        g_r2c = SE2(rot2(pi/4-pi/4*(t-2))); psi = 0; scale = 1+0.3-0.3*(t-2);
    end
    if t<=1.5
        g_r = SE2([2.5,2.5,0]+ones(1,3)/2*t);
    else
        g_r = SE2([2.5,2.5,0]+ones(1,3)/2*(3-t));
    end
    [g_r2e,p_r2m_hat,th_r] = cm.gfkine(g_r2c,psi,scale);
    g_c = g_r*g_r2c; g_e = g_r*g_r2e; p_m_hat = h2e(g_r.T*e2h(p_r2m_hat));
    cm.animate(g_c.q,g_e.q,th_r(:,2:end));
    set(h,'xdata',p_m_hat(1,[1:end,1]),'ydata',p_m_hat(2,[1:end,1]));
    if video_on
        frame = getframe;
        frame.cdata = imresize(frame.cdata,[480,640]);
        writeVideo(writerObj,frame);
    end
end
if video_on
    close(writerObj);
end

% q0 = [0,0,0,0,0,0];
% q1 = [1,0,0,-1,1,2];
% q2 = [1,1,0,1,2,1];
% [qq,~,~,tq] = calctraj([q0;q1;q2],0.1*ones(size(q0)),0.1,1);
% tic;
% playspeed = 2;
% while toc<tq(end)/playspeed
%     tnow = toc*playspeed;
%     % choose via point in time 'tnow'
%     q = interp1(tq,qq,tnow); 
%     % update object
%     cm.animate(q);
%     drawnow
% end