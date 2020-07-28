close all
clear

radius = 2; 
l = Lidar(radius,'name','lidar1','hlim',[-1/2,0]);
m = 3; link = ones(m,1)*0.8; edge = [1,.8,.5]; mh = [0.2,0.2,1];
r = PlanarRevolute(link,'name','rob1','height',1);


q0 = [0,0,0,0,0,1/2,0,0,0];
q1 = [-1,1,2,1,0,1/2,0,0,1];
q2 = [1,2,1,1,1,1/2,0,0,0];
[qq,~,~,tq] = mstraj_([q0;q1;q2],0.1*ones(size(q0)),0.1,1);

figure
d = 3;
hr = r.plot(q0(1:3),q0(4:end),'plat','workspace',[-1 1 -1 1 0 2]*2*radius,'hgsize',3,'lkthick',1,'dim',d);hold on
hl = l.plot(q0(4:end),'licolor','g','dim',d,'nodetect');

% write video
video_on = false;
if video_on
    respath = 'D:\Users\Woody\Documents\MATLAB\';
    videoname = [respath 'lidar_demo'];
    writerObj = VideoWriter(videoname);
    open(writerObj);
end

tic;
playspeed = 3;
while toc<tq(end)/playspeed
    tnow = toc*playspeed;
    % choose via point in time 'tnow'
    q = interp1(tq,qq,tnow); 
    % update object
    r.animate(q(1:3),q(4:end),hr.group);
    l.animate(q(4:end),hl.group);
    % video
    if video_on
        f = getframe(gcf);
        f.cdata = imresize(f.cdata,[480,640]);
        writeVideo(writerObj,f);
    end
    drawnow
end
toc
if video_on
    close(writerObj);
end