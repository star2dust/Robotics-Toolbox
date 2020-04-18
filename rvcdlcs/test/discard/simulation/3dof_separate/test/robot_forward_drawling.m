close all; clear;

%% parameters
% link length
l0 = 0.4; l1 = 0.5; l2 = 0.5; l3 = 0.5; l4 = 0.1;
% choose q according to type of joints
q1 = [0,0,0]';
q2 = [l1,0,0]';
q3 = [l1+l2,0,0]';
w1 = [0,0,1]';
w2 = w1; w3 = w1;
% platform
v1 = [1,0,0]';
v2 = [0,1,0]';
% joint twists (plt=>1:3,slk=>4:6)
xi1 = [v1;zeros(3,1)];
xi2 = [v2;zeros(3,1)];
xi3 = [-skew(w1)*q1;w1];
xi4 = [-skew(w1)*q1;w1];
xi5 = [-skew(w2)*q2;w2];
xi6 = [-skew(w3)*q3;w3];
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
rod1 = rod*l1; rod2 = rod*l2; rod3 = rod*l3; cub0 = cub*l0; grp4 = grp*l4;
slk1 = g_sl0{4}*e2h(rod1);
slk2 = g_sl0{5}*e2h(rod2);
slk3 = g_sl0{6}*e2h(rod3);
plt = g_sl0{3}*e2h(cub0);
eft = g_st0*e2h(grp4);
% convas
figure
subplot(1,1,1);
axis([-5 5 -5 5]);
hold on
plot3(slk1(1,:),slk1(2,:),slk1(3,:),'b');
plot3(slk2(1,:),slk2(2,:),slk2(3,:),'b');
plot3(slk3(1,:),slk3(2,:),slk3(3,:),'b');
plot3(plt(1,:),plt(2,:),plt(3,:),'b');
plot3(eft(1,:),eft(2,:),eft(3,:),'b');
% plot robot given th Fm
th = [-1,-0.2838,-0.6435]';
Fm = [1,2,1]';
xi = {xi1,xi2,xi3,xi4,xi5,xi6};
figure
subplot(1,1,1);
axis([-5 5 -5 5]);
hold on
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
end
slk1 = g_sl{4}*e2h(rod1);
slk2 = g_sl{5}*e2h(rod2);
slk3 = g_sl{6}*e2h(rod3);
plt = g_sl{3}*e2h(cub0);
eft = g_st*e2h(grp4);
plot3(slk1(1,:),slk1(2,:),slk1(3,:),'b');
plot3(slk2(1,:),slk2(2,:),slk2(3,:),'b');
plot3(slk3(1,:),slk3(2,:),slk3(3,:),'b');
plot3(plt(1,:),plt(2,:),plt(3,:),'b');
plot3(eft(1,:),eft(2,:),eft(3,:),'b');