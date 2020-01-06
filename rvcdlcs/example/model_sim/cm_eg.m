close all
clear

nrob = 4; nlink = 2; radius = .5;
cm = CooperativeManipulation(nrob,nlink,radius,'name','cm');
qe = [ones(4,2)*5/2,[3*pi/4;-3*pi/4;-pi/4;pi/4]+3*pi/8]+radius*[0.7,-0.7,0;
    0.7,0.7,0;
    -0.7,0.7,0;
    -0.7,-0.7,0;];
qr = [-.5;-.5;-.5;-.5;]*2;
cm.plot(qe,qr,'workspace',[0 10 0 10 0 10]/2,'noframe');
q0 = [0,0,0,0,0,0];
q1 = [1,0,0,-1,1,2];
q2 = [1,1,0,1,2,1];
[qq,~,~,tq] = calctraj([q0;q1;q2],0.1*ones(size(q0)),0.1,1);
tic;
playspeed = 2;
while toc<tq(end)/playspeed
    tnow = toc*playspeed;
    % choose via point in time 'tnow'
    q = interp1(tq,qq,tnow); 
    % update object
    cm.animate(q);
    drawnow
end