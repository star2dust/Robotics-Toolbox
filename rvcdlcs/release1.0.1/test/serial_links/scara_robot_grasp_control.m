% Two SCARA fingers grasp an object - controller design
% Reference book: Richard M. Murray-A Mathematical Introduction to Robotic Manipulation-CRC Press (1994).
close all
clear

%% axis directions
ex = [1,0,0,0,0,0]';
ey = [0,1,0,0,0,0]';
ez = [0,0,1,0,0,0]';
ox = [0,0,0,1,0,0]';
oy = [0,0,0,0,1,0]';
oz = [0,0,0,0,0,1]';
%% object cuboid: wx*wy*wz (Fig 5.17 in page 261)
syms obj_wx obj_wy obj_wz obj_mass obj_height real
% obj_wx = 2; obj_wy = 1; obj_wz = 1; obj_mass = 1; obj_height = 1;
object_data = cubstruct(obj_mass,[obj_wx,obj_wy,obj_wz],SE3(0,0,obj_height));
%% SCARA construction and forward kinematics (page 108)
rob_length = sym('rl',[1,4],'real'); rob_mass = sym('rm',[1,4],'real');
% rob_length = [1,1,1,0.5]*2; rob_mass = [1,1,1,0.5];
% choose q and w according to type of joints (in spatial frame see Fig. 3.3)
rob_axis_q = [[0,0,0]', [rob_length(1),0,0]', [rob_length(1)+rob_length(2),0,0]', [rob_length(1)+rob_length(2),0,0]'];
rob_axis_w = [[0,0,1]', [0,0,1]', -[0,0,1]', [0,0,1]'];
% reference config for each joint (at center of mass)
g_sl0(1) = SE3(rob_length(1)/2,0,rob_length(3));
g_sl0(2) = SE3(rob_length(1)+rob_length(2)/2,0,rob_length(3));
g_sl0(3) = SE3(rob_length(1)+rob_length(2),0,rob_length(3)*1.5);
g_sl0(4) = SE3(rob_length(1)+rob_length(2)+rob_length(4)/2,0,rob_length(3));
g_st0 = SE3(rob_length(1)+rob_length(2)+rob_length(4),0,rob_length(3));
% make links
for i = 1:length(rob_axis_w)
    % scara robot => RRPR
    if i==3
        robot_data.link(i) = linkstruct(rob_mass(i), rob_length(i), 'P', rob_axis_q(:,i), rob_axis_w(:,i), g_sl0(i));
    else
        robot_data.link(i) = linkstruct(rob_mass(i), rob_length(i), 'R', rob_axis_q(:,i), rob_axis_w(:,i), g_sl0(i));
    end
end
robot_data(2) = robot_data;
% robot initial link xyzdata
nrob = length(robot_data);
for i=1:nrob
    robot_data(i).base = SE3(0,0,0);
    robot_data(i).xyzdata = robot_data(i).base.t;
    for j=1:length(robot_data(i).link)
        robot_data(i).xyzdata = [robot_data(i).xyzdata, h2e(robot_data(i).base.T*robot_data(i).link(j).refconfig.T*e2h(robot_data(i).link(j).vertmat))];
    end
end
%% soft-finger grasp => get G & B (fixed)(page 241)
% config g_po (SE3) => distance between P and O
% g_po = object_data.config;
% config contact g_oc (SE3)
object_data.g_oc(1) = SE3(transl(-object_data.width(1)/2,0,0));
object_data.g_oc(2) = SE3(transl(object_data.width(1)/2,0,0)*trotz(pi));
% grasp map => soft-finger (NOTICE: x axis as main direction)
object_data.B_c = [ex,ey,ez,ox];
object_data.G = [];
H = [ex,ey,ez,oy,oz];
for i=1:nrob
    object_data.G = [object_data.G, H'*Ad(inv(object_data.g_oc(i)))'*object_data.B_c];
end
object_data.G = blkdiag(object_data.G,eye(2));
%% two scara fingers grasping a box => set initial config (page 261)
% config g_sp (SE3)
syms b real
% b = 4; % distance between P and S
g_sp(1) = SE3(transl(b,0,0));
g_sp(2) = SE3(transl(b,0,0)*trotz(pi));
for i=1:nrob
    % robot base and tool (in spatial frame)
    robot_data(i).base = inv(g_sp(i)); % g_ps
    robot_data(i).tool = object_data.config*object_data.g_oc(i); % g_pc = g_po*g_oc
    % relative config g_sc
    robot_data(i).g_sc = robot_data(i).base.inv*robot_data(i).tool; % assuming that g_fc = SE3(eye(4))
end
% reset robot xyzdata according to initial object config
rob_th0 = sym('rth',[4,2],'real');
% rob_th0 = ikine4([robot_data(i).link.length],robot_data(i).base.inv*robot_data(i).tool);
for i=1:nrob
    robot_data(i).xyzdata = robot_data(i).base.t;
    for j=1:length(robot_data(i).link)
        robot_data(i).link(j).curconfig = gSE3({robot_data(i).link.twist},rob_th0(:,i),0,j)*robot_data(i).link(j).refconfig;
        robot_data(i).xyzdata = [robot_data(i).xyzdata, h2e(robot_data(i).base.T*robot_data(i).link(j).refconfig.T*e2h(robot_data(i).link(j).vertmat))];
    end
end
%% two scara fingers grasping a box => get J_h (page 261) and derivative of J_h^-1*G^T
rob_thd0 = sym('rthd',[4,2],'real');
% Jacobian for a SCARA robot (page 139)
for i=1:nrob
   robot_data(i).J_sf_s(:,1) = robot_data(i).link(1).twist;
   for j=2:length(robot_data(i).link)
       robot_data(i).J_sf_s(:,j) = Ad(gSE3({robot_data(i).link.twist},rob_th0(:,i),0,j-1))*robot_data(i).link(j).twist;
   end
end
% Adjoint tranformation associated with g_sf(g_sc)
object_data.J_h = [];
for i=1:nrob
    object_data.J_h = blkdiag(object_data.J_h, object_data.B_c'*Ad(inv(robot_data(i).g_sc))*robot_data(i).J_sf_s);
end
K1 = [1,1,0,1;0,0,0,0]; K2 = [0,0,0,0;1,1,0,1]; 
object_data.J_h = [object_data.J_h;K1,K2];
object_data.Jh_inv = (object_data.J_h'*object_data.J_h)^-1*object_data.J_h';
object_data.Jh_dot = sym(zeros(10,8));
object_data.Jh_dot(:,1) = zeros(10,1);
object_data.Jh_dot(:,2) = rob_length(1)*rob_thd0(1,1)*[cos(rob_th0(1,1));sin(rob_th0(1,1));zeros(8,1)];
object_data.Jh_dot(:,3) = zeros(10,1);
object_data.Jh_dot(:,4) = [rob_length(1)*rob_thd0(1,1)*cos(rob_th0(1,1))+rob_length(2)*(rob_thd0(1,1)+rob_thd0(2,1))*cos(rob_th0(1,1)+rob_th0(2));rob_length(1)*rob_thd0(1,1)*sin(rob_th0(1,1))+rob_length(2)*(rob_thd0(1,1)+rob_thd0(2,1))*sin(rob_th0(1,1)+rob_th0(2,1));zeros(8,1)];
object_data.Jh_dot(:,5) = zeros(10,1);
object_data.Jh_dot(:,6) = rob_length(1)*rob_thd0(1,2)*[zeros(4,1);cos(rob_th0(1,2));sin(rob_th0(1,2));zeros(4,1)];
object_data.Jh_dot(:,7) = zeros(10,1);
object_data.Jh_dot(:,8) = [zeros(4,1);rob_length(1)*rob_thd0(1,2)*cos(rob_th0(1,2))+rob_length(2)*(rob_thd0(1,2)+rob_thd0(2,2))*cos(rob_th0(1,2)+rob_th0(2,2));rob_length(1)*rob_thd0(1,2)*sin(rob_th0(1,2))+rob_length(2)*(rob_thd0(1,2)+rob_thd0(2,2))*sin(rob_th0(1,2)+rob_th0(2,2));zeros(4,1)];
%% intial figure
% figure
% view(3); hold on; grid;
% axis([-4 4 -4 4 0 4])
% xlabel('X'); ylabel('Y'); zlabel('Z'); 
% for i=1:nrob
%     robot_data(i).linkline = line(robot_data(i).xyzdata(1,:),robot_data(i).xyzdata(2,:),robot_data(i).xyzdata(3,:),'LineWidth', 3);
%     robot_data(i).frameC = robot_data(i).tool.plot('color', 'r', 'arrow','frame', ['C_',num2str(i)]);
%     robot_data(i).frameS = robot_data(i).base.plot('color', 'b', 'arrow','frame', ['S_',num2str(i)]);
% end
% object_data.surface = patch('Faces',object_data.faces,'Vertices',(object_data.config*object_data.vertices')','FaceColor', 'y', 'FaceAlpha', 0.5);
% object_data.frameO = object_data.config.plot('color', 'k', 'arrow','frame', 'O');
%% generate path of a circle
obj_path_ds = [0,0]';
thvia = 0:pi/30:pi/6;
obj_path_via = [cos(thvia);sin(thvia)]+obj_path_ds;
obj_path = [obj_path_via; zeros(1,numcols(obj_path_via))];
%% generate trajectory
% add reference config to the start of the path
obj_path = [object_data.config.t, obj_path+object_data.config.t];
% the second argument is the maximum speed in the x-, y- and z-directions, the fourth argument is the initial coordinate followed by the sample interval and the acceleration time. 
obj_traj = mstraj(obj_path(:,2:end)', [0.5 0.5 0.5], [], obj_path(:,1)',0.02, 0.2);
% the orientation is not varying
% obj_traj = [obj_traj,zeros(size(obj_traj))];
% plot3(obj_traj(:,1),obj_traj(:,2),obj_traj(:,3));
% % get Cartesian trajectory 
obj_SE3traj = SE3(obj_traj);
% % inverse kinematics
% for i=1:2
%     th_traj{i} = ikine4([robot_data(i).link.length],robot_data(i).base.inv*obj_SE3traj*g_oc(i));
%     tool_traj{i} = fkine4([robot_data(i).link.length],th_traj{i});
%     scaraplot(robot_data(i),th_traj{i},[-4 4 -4 4 0 4]);
% end
%% robot control
dt = 0.05; 
[xarray, xdarray, xddarray, tf] = calctraj([obj_SE3traj.transl,obj_SE3traj.torpy],[1 1 1 0.5 0.5 0.5]*0.5,dt,0.2);
% test tharray by scaraplot(l,xi,g_sl0,tharray); 
xxd0 = [xarray(1,:), xdarray(1,:)]; % q0 can be either row or column vector
for i=1:nrob
    th0{i} = ikine4([robot_data(i).link.length],robot_data(i).base.inv*SE3(xarray(1,1:3))*SE3.rpy(xarray(1,4:6))*object_data.g_oc(i));
    ththd0{i} = [th0{i},zeros(size(th0{i}))];
end
qqd0 = [xxd0, ththd0{:}];
[tarray,qqdarray] = ode45(@(t,qqd) graspqdot(t, qqd, xarray, xdarray, xddarray, 0:dt:tf, robot_data, object_data),[0 tf], qqd0);
%% final figure
nx = size(xarray,2);
xtraj = qqdarray(1:nx);
xdtraj = qqdarray(nx+1:2*nx);
nth = 0; idx = 2*nx;
for i=1:nrob
    idx = idx + 2*nth;
    nth = length(robot_data(i).link);
    thtraj{i} = qqdarray(:,idx+1:idx+nth);
    thdtraj{i} = qqdarray(:,idx+nth+1:idx+2*nth);
end
for k=1:size(xtraj,1)
    % update config
    object_data.config = SE3(xtraj(k,1:3))*SE3.rpy(xtraj(k,4:6));
    for i=1:nrob
        robot_data(i).xyzdata = robot_data(i).base.t;
        for j=1:length(robot_data(i).link)
            robot_data(i).link(j).curconfig = gSE3({robot_data(i).link.twist},thtraj{i}(k,:),0,j)*robot_data(i).link(j).refconfig;
            robot_data(i).xyzdata = [robot_data(i).xyzdata, robot_data(i).base*robot_data(i).link(j).curconfig*robot_data(i).link(j).vertmat];
        end
    end
    % update figure
    set(object_data.surface, 'Vertices', (object_data.config*object_data.vertices')');
    for i=1:nrob
        set(robot_data(i).linkline, 'XData', robot_data(i).xyzdata(1,:), 'YData', robot_data(i).xyzdata(2,:), 'ZData', robot_data(i).xyzdata(3,:));
    end
    drawnow
end
% D-H params (page 114, toolbox page 216)
% if you use D-H params, the i-th frame should be in the end of the i-th link
% g = SE3(transl([0,0,d])*trotz(th)*transl([a,0,0])*trotx(alpha))
% see also https://robotics.stackexchange.com/questions/4364/denavit-hartenberg-parameters-for-scara-manipulator
link = [ Revolute('d',rob_l3,'a',rob_l1), Revolute('a', rob_l2, 'alpha', pi), Prismatic('alpha', pi), Revolute('a', rob_l4)];
% SCARA construction and forward kinematics (toolbox page 219)
% SerialLink object cannot use list
% figure
% view(3); hold on
% axis([-1 1 -1 1 -1 1]*3);
% object_fig.surface = patch('Faces',object_data.faces,'Vertices',transl(g_po*SE3(object_data.vertices)),'FaceColor', 'y', 'FaceAlpha', 0.5);
% object_fig.frameO = object_data.config.plot('color', 'k', 'arrow','frame', 'O');
for i=1:2
    scara{i} = SerialLink(link, 'name', ['scara',num2str(i)]);
    scara{i}.base = inv(g_sp(i));
    % th0 = scaraikine(l,g_sp(i)*g_po*g_oc(i));
    % scara{i}.plot(th0,'workspace',[-1 1 -1 1 -1 1]*3, 'tilesize', 0.5);
end
% hold off