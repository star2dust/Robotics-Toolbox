close all
clear

obs_r = 1; obs_h = 1;
obs(1) = Cylinder(obs_r,obs_h,'name','obs1');
obs(2) = Cylinder(obs_r,obs_h,'name','obs2');

q_obs(1,:) = [1,1,0.5,0,0,0];
q_obs(2,:) = [3,3,0.5,0,0,1];

rob = Cylinder(1,1,'name', 'rob');
q_rob = [0,3,0.5,0,0,0];

q0 = [0,3,0.5,0,0,0];
q1 = [2,2,0.5,0,0,1];
q2 = [3,0,0.5,0,0,0];

[qq,~,~,tq] = mstraj_([q0;q1;q2],0.1*ones(size(q0)),0.1,1);

figure; 
for i=1:2
   obs(i).plot(q_obs(i,:),'workspace',[-1,5,-1,5,0,6],'facecolor','b'); 
   hold on
end
rob.plot(q_rob,'frame'); 
plot3(qq(:,1),qq(:,2),qq(:,3),'r-');
rotate3d on
hold off

tic;
playspeed = 2;
while toc<tq(end)/playspeed
    tnow = toc*playspeed;
    % choose via point in time 'tnow'
    q = interp1(tq,qq,tnow); 
    % update object
    rob.animate(q);
    drawnow
end