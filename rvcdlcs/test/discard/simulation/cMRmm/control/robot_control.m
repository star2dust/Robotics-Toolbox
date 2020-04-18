close all
clear

import SE2.*
import mpr.*
import iris.drawing.*

%% reference trajectory
% trajectory
load('shortestpath2.mat','nodez','nodepoly','edgez','noderoute','polyroute');
dt = 0.5; tacc = 2; dgmax = 1;
dfnz = diff(nodez(noderoute,1:2)); thnz = [0;atan2(dfnz(:,2),dfnz(:,1))];
thnz = zeros(size(nodez(noderoute,1)));
gcrnode = [nodez(noderoute,1:2),thnz];
[gcrarray, dgcrarray, ddgcarray, tarray] = calctraj(gcrnode,dgmax*ones(1,size(gcrnode,2)),dt,tacc);
% polychange
tchange = [0,4,11,13,18,25]/5*4;
polychange = [1,1;1,4;4,3;3,3;3,2;2,2];
%% object and intial grasps
lcx = 0.8; lcy = 0.8; % size of object
gc_r0 = gcrarray(1,:)';
cub = Cuboid2(1,gc_r0,[lcx,lcy]);
% geometry of the grasp
n = 4; % number of grasps
g_ce0 = [[1,1,-1,-1]*lcx/2;
    [-1,1,1,-1]*lcy/2;
    3*pi/4,-3*pi/4,-pi/4,pi/4];
for i=1:n
    ge0(:,i) = SE2toPose(SE2(gc_r0)*SE2(g_ce0(:,i))); % initial end-effector pose
end

%% robot parameters
% mR manipulator (m dof) 
m = 3; 
% link length (arm=>la,platform=>lm,end-effector=>le)
la = ones(m,1)/m; lm = 0.45; le = 0.15;
% get twist
xi = MmRtwist(la);
% robot states
th_tilde0 = kron(ones(m-1,4),-pi/4);
delta = [-ones(1,m-1);eye(m-1)];
for i=1:n
    th0(:,i) = [ge0(3,i)-sum(th_tilde0(:,i));th_tilde0(:,i)]; % initial joints
    g_em0(:,i) = [-mRFkine(delta*th_tilde0(:,i),la);-ge0(3,i)];
    g_cm0(:,i) = SE2toPose(SE2(g_ce0(:,i))*SE2(g_em0(:,i)));
    gm0(:,i) = SE2toPose(SE2(gc_r0)*SE2(g_cm0(:,i))); % initial platform pose
end
%% obstacles
xo1 = [3.75,4.75,0];
xo2 = [8.25,3,0];
obs(1) = Cuboid2(xo1,[1.5,3.5]-lm/2);
obs(2) = Cuboid2(xo2,[1.5,1]-lm/2);
obs_pts = {obs(1).verts',obs(2).verts'};
%% range
range.lb = [0;0]-lm/2;
range.ub = [10;10]+lm/2;
%% visualize trajectory
vis_traj_anime = 1; vis_traj_on = 0;
if vis_traj_on
    if ~vis_traj_anime
        draw_region_2d(nodepoly(polyroute),{obs.verts},range);
        plot(gcrarray(:,1),gcrarray(:,2),'b-', 'LineWidth', 2);
        plot(gcrarray(end,1),gcrarray(end,2),'bd', 'LineWidth', 2);
    else
        for t = 0:0.1:25
            gc_hat = interp1(tarray,gcrarray,t);
            [~,gcidx] = min(abs(t-tarray));
            plot(gc_hat(1),gc_hat(2),'bo'); hold on
            plot(gcrarray(1:gcidx,1),gcrarray(1:gcidx,2),'r-');
            plot(gcrarray(end,1),gcrarray(end,2),'rd')
            if min(abs(t-tchange))<=0.1
                [~,polyidx] = min(abs(t-tchange));
            end
            draw_region_2d(nodepoly(polychange(polyidx,2)),{obs.verts},range);
            pause(0.1)
            hold off
        end
    end
end



%% bounded constraints
% g_ce range
g_ce_lb = g_ce0-[0.1;0.1;pi/4]/2;
g_ce_ub = g_ce0+[0.1;0.1;pi/4]/2;
% th2 range
th_tilde_ub = kron(ones(m-1,4),-asin(0.1));
th_tilde_lb = kron(ones(m-1,4),-pi/2);
% z range
d_z = g_cm0(1:2,:); z0 = kron(ones(1,4),[0;0;1]);
z_lb = kron(ones(1,4),[-0.2;-0.2;0.5]);
z_ub = kron(ones(1,4),[0.2;0.2;1]);
for i=1:n
    D_z{i} = [eye(2),d_z(:,i)];
end
%% visualize initials
vis_init_on = 1;
if vis_init_on
    fh = figure;
    hmap = draw_region_2d(nodepoly(polychange(1,2)),{obs.verts},range);
    plot(gcrarray(:,1),gcrarray(:,2),'r-');
    plot(gcrarray(end,1),gcrarray(end,2),'rd');
    hc.cub = patch('vertices', cub.verts, 'faces', cub.faces, 'facecolor', 'y', 'facealpha', 0.5);
    gc_SE2 = SE2(gc_r0); gc_SE2.plot;
%     hc.gc = SE2plot(gc_r0,'b-',0.5); %set(hc.gc,'visible','off');
%     hc.gc_r = SE2plot(gc_r0,'r-',0.5); %set(hc.gc,'visible','off');
    for i=1:n      
        hm(i).rob = MmRplot(ge0(:,i),th0(:,i),xi,[lm;la;le],'mbbbbbbbr');
        hm(i).ge = SE2plot(ge0(:,i),'c-',0.5); set(hm(i).ge,'visible','off');
    end
    hz = plot(gc_r0(1)+d_z(1,:),gc_r0(2)+d_z(2,:),'gd');
    set(fh, 'units', 'inches', 'position', [5 5 10 9])
    hold off
end

%% intersection constraints
for i=1:4
    % linear programming [X;Y]=f*[g_ce(1:2);z]
    R1{i} = rot2(g_ce_lb(3,i))'; f{i} = [R1{i},-R1{i}*D_z{i}]';
    [~,X_lb(:,i)] = linprog(f{i}(:,1),[],[],[],[],[g_ce_lb(1:2,i);z_lb(:,i)],[g_ce_ub(1:2,i);z_ub(:,i)]);
    [~,X_ub(:,i)] = linprog(-f{i}(:,1),[],[],[],[],[g_ce_lb(1:2,i);z_lb(:,i)],[g_ce_ub(1:2,i);z_ub(:,i)]);
    [~,Y_lb(:,i)] = linprog(f{i}(:,2),[],[],[],[],[g_ce_lb(1:2,i);z_lb(:,i)],[g_ce_ub(1:2,i);z_ub(:,i)]);
    [~,Y_ub(:,i)] = linprog(-f{i}(:,2),[],[],[],[],[g_ce_lb(1:2,i);z_lb(:,i)],[g_ce_ub(1:2,i);z_ub(:,i)]);
    % initials
    X0(i) = f{i}(:,1)'*[g_ce0(1:2,i);z0(:,i)];
    Y0(i) = f{i}(:,2)'*[g_ce0(1:2,i);z0(:,i)];
    TH0 = th_tilde0;
    % modify joint space to convex space 
    J1{i} = [-la(1)*sin(g_ce0(3,i)-TH0(i))-la(2)*sin(g_ce0(3,i)), la(1)*sin(g_ce0(3,i)-TH0(i));
        la(1)*cos(g_ce0(3,i)-TH0(i))+la(2)*cos(g_ce0(3,i)),-la(1)*cos(g_ce0(3,i)-TH0(i))];
end
X_ub = -X_ub; Y_ub = -Y_ub;
TH_lb = th_tilde_lb; TH_ub = th_tilde_ub;
% plot original/modified joint space
vis_jotspc_on = 0; vis_org = 0;
if vis_jotspc_on
    for i=1:4
        figure
        [X,Y,TH] = meshgrid(X_lb(:,i)-.01:.01:X_ub(:,i)+.01,Y_lb(:,i)-.01:.01:Y_ub(:,i)+.01,TH_lb(:,i)-.01:.01:TH_ub(:,i)+.01);
        cx = max(-X+X_lb(:,i),X-X_ub(:,i));
        cy = max(-Y+Y_lb(:,i),Y-Y_ub(:,i));
        cz = max(-TH+TH_lb(:,i),TH-TH_ub(:,i));
        cth = max(g_ce0(3,i)-g_ce_lb(3,i)-TH-pi/2,-(g_ce0(3,i)-g_ce_lb(3,i)-TH));
        if vis_org
            Fkx{i} = R1{i}(1,1)*(la(1)*cos(g_ce0(3,i)-TH)+la(2)*cos(g_ce0(3,i)))+R1{i}(1,2)*(la(1)*sin(g_ce0(3,i)-TH)+la(2)*sin(g_ce0(3,i)));
            Fky{i} = R1{i}(2,1)*(la(1)*cos(g_ce0(3,i)-TH)+la(2)*cos(g_ce0(3,i)))+R1{i}(2,2)*(la(1)*sin(g_ce0(3,i)-TH)+la(2)*sin(g_ce0(3,i)));
            ints = max(-(X-Fkx{i}),-(Y-Fky{i}));
        else            
            % tangent plane
            intA0 = [eye(2),-R1{i}*J1{i}]; intb0 = intA0*[X0(i);Y0(i);g_ce0(3,i);TH0(i)];
            ints = max(-intA0(1,1)*X-intA0(1,2)*Y-intA0(1,3)*g_ce0(3,i)-intA0(1,4)*TH+intb0(1),-intA0(2,1)*X-intA0(2,2)*Y-intA0(2,3)*g_ce0(3,i)-intA0(2,4)*TH+intb0(2));
        end
        cons = max(ints,max(cx,max(cy,max(cz,cth))));
        facevert = isosurface(X,Y,TH,cons,0);
        pat = patch(facevert);
        isonormals(X,Y,TH,cons,pat)
        set(pat,'facecolor',[0 .5 1],'edgecolor','none');hold on
        view(150,30),axis image,grid on
        ylabel('Y');xlabel('X');zlabel('TH');
        camlight
        lighting gouraud
        plot3(X0(i),Y0(i),TH0(i),'ro');
    end
end
% manipulability
negmu = @(x) -mRMu(delta*x,la);
for i=1:n
    th_bar(:,i) = fmincon(negmu,th_tilde0(:,i),[],[],[],[],th_tilde_lb(:,i),th_tilde_ub(:,i));
end



%% comunication
% Laplacian
D = [-1,0,0,1,-1;
    1,-1,0,0,0;
    0,1,-1,0,1;
    0,0,1,-1,0;];
A = [0,1,1,1;
    1,0,1,0;
    1,1,0,1;
    1,0,1,0;];
L = D*D';

% write video
video_on = 0;
if video_on
    videoname = 'robot_control_v1.1';
    writerObj = VideoWriter(videoname);
    open(writerObj);
end

%% test
% q = [ 0.3500    0.4500   -0.3500   -0.4500;
%    -0.4500    0.3500    0.4500   -0.3500;
%     2.3562   -2.3562   -0.7854    0.7854;
%    -0.7818   -0.8336   -0.7818   -0.8336];
% z = [-0.0015   -0.0066    0.0014    0.0067;
%     0.0001    0.0105    0.0000   -0.0105;
%     0.9822    0.9848    0.9824    0.9848];
% ge = [6.3563    6.2590    5.4587    5.5559;
%     1.3426    2.1430    2.0457    1.2454;
%     2.6015   -2.1109   -0.5401    1.0307];
% gc = [5.9075;
%     1.6942;
%     0.2453];
% t0 = 11.69;
% q0 = [g_ce0;th_tilde0];

%% simulation
q = [g_ce0;th_tilde0]; q0 = q; z = z0; ge = ge0; gc_hat = kron(ones(1,n),gc_r0); idx0 = 1; t0 = 0;
lambda = zeros(size(z)); 
eta = zeros(3,n);
q_lb = [g_ce_lb;th_tilde_lb];
q_ub = [g_ce_ub;th_tilde_ub];
% control loop
dt = 0.01; tf = tarray(end); polyidx = idx0;
for t=t0:dt:tf
    % trajectory
    if t<tf
    gc_r = interp1(tarray,gcrarray,t);
    dgc_r = interp1(tarray,dgcrarray,t);
    end
    if min(abs(t-tchange))<=dt
        [~,polyidx] = min(abs(t-tchange));
    end
    % optimization
    g_ce = q(1:3,:); th_tilde = q(4:end,:); S = rot2(pi/2);
    wf = 1; wm = 1; alpha = 5; sgn_eta = zeros(size(g_ce)); L_lambda = zeros(size(z));
    for i=1:n
        g_ce_SE2(i) = SE2(g_ce(:,i)); [fk(:,i),J{i}] = mRFkine(delta*th_tilde(:,i),la);
        hq(:,i) = g_ce_SE2(i).t-g_ce_SE2(i).R*fk(:,i);
        nabla_hq{i} = [eye(2),-S*g_ce_SE2(i).R*fk(:,i),-g_ce_SE2(i).R*J{i}*delta]';
        uq(:,i) = wf*nabla_hq{i}*(hq(:,i)-D_z{i}*z(:,i))+[eta(:,i);wm*(th_tilde(:,i)-th_bar(:,i))];
        for j=1:n
            L_lambda(:,i) = L_lambda(:,i) + A(i,j)*(lambda(:,i)-lambda(:,j));
            sgn_eta(:,i) = sgn_eta(:,i) + A(i,j)*tanh(100*(eta(:,i)-eta(:,j)));
            %sgn_lambda(:,i) = sgn_lambda(:,i) + A(i,j)*sign(lambda(:,i)-lambda(:,j));
        end
        uz(:,i) = -wf*D_z{i}'*(hq(:,i)-D_z{i}*z(:,i))+L_lambda(:,i);
        % projection
        consA = []; consb = [];
        % obstacle free region
%         D_z_inflat{i} = D_z{i}+[zeros(2),d_z(:,i)/norm(d_z(:,i))*(lm+dt*norm(dgc_r))];
%         poly{i} = polytope(obs_pts,gc(1:2),range);
%         Aobs = poly{i}.A*SE2(gc).R*D_z_inflat{i}; bobs = poly{i}.b-poly{i}.A*SE2(gc).t;
%         for j=2:-1:1
            D_z_inflat{i} = D_z{i};%+[zeros(2),d_z(:,i)/norm(d_z(:,i))*(lm/2+dt*norm(dgc_r))];
            Aobs = nodepoly(polychange(polyidx,2)).A*SE2(gc_hat(:,i)).R*D_z_inflat{i}; bobs = nodepoly(polychange(polyidx,2)).b-nodepoly(polychange(polyidx,2)).A*SE2(gc_hat(:,i)).t;
%             if Aobs*z(:,i)-bobs<=0
%                 break
%             end
%         end
        consA = [consA;zeros(size(Aobs,1),m+2),Aobs];
        consb = [consb;bobs];
        % tangent plane
        g_ce0_SE2(i) = SE2(q0(1:3,i)); [fk0(:,i),J0{i}] = mRFkine(delta*th_tilde0(:,i),la);
        nabla_hq0{i} = [eye(2),-S*g_ce0_SE2(i).R*fk0(:,i),-g_ce0_SE2(i).R*J0{i}*delta]';
        Atan = rot2(q0(3,i))'*[nabla_hq0{i}',-D_z{i}]; btan = Atan*[q0(:,i);z0(:,i)];
        consA = [consA;-Atan]; consb = [consb;-btan];
        % angle 
        U = triu(ones(m));
        Aang = [zeros(m,2),ones(m,1),U'*delta,zeros(m,3);
            zeros(m,2),-ones(m,1),-U'*delta,zeros(m,3)];
        bang = kron([pi/2+q0(3,i);-q0(3,i)],ones(m,1));
        consA = [consA;Aang]; consb = [consb;bang];
        dqz(:,i) = lsqlin(eye(size(q,1)+size(z,1)),[q(:,i)-uq(:,i);z(:,i)-uz(:,i)],consA,consb,[],[],[q_lb(:,i);z_lb(:,i)],[q_ub(:,i);z_ub(:,i)])-[q(:,i);z(:,i)];
        dq(:,i) = dqz(1:m+2,i); dz(:,i) = dqz(end-2:end,i); dg_ce(:,i) = dqz(1:3,i);
    end
%     dq = -uq; dz = -uz; dg_ce = dq(1:3,:);
    dlam = kron(L,eye(3))*(z(:)+dz(:));
    deta = g_ce(:)+dg_ce(:)-alpha*sgn_eta(:);
    % tracking
    b = [1;0;1;0]; M = diag(b)+L; gamma = 5;
    dgc_hat = kron(M,eye(3))^-1*(-gamma*(kron(M,eye(3))*gc_hat(:)-kron(b,eye(3))*gc_r')+kron(b,eye(3))*dgc_r');
    % renew
    q(:) = q(:) + dq(:)*dt;
    z(:) = z(:) + dz(:)*dt;
    lambda(:) = lambda(:) + dlam*dt;
    eta(:) = eta(:) + deta*dt;
    gc_hat(:) = gc_hat(:) + dgc_hat*dt;
    % plot
    gc = kron(ones(1,n),eye(3))*gc_hat(:)/n;
    gc_SE3 = SE3([gc(1:2);0])*SE3.rpy([0,0,gc(3)]);
    gc_r_SE3 = SE3([gc_r(1:2)';0])*SE3.rpy([0,0,gc_r(3)]);
    gc_SE2=SE2(gc);
    if mod(t,0.01)==0
        gc_SE2.animate;
        set(hc.gc,'matrix',gc_SE3.T);
        set(hc.gc_r,'matrix',gc_r_SE3.T);
        cub.updatePose(gc);
        set(hc.cub,'vertices', cub.verts, 'faces', cub.faces);
        for i=1:n
            ge(:,i) = SE2toPose(SE2(gc_hat(:,i))*SE2(q(1:3,i)));
            th(:,i) = [ge(3,i)-sum(q(4:end,i));q(4:end,i)];
            hm(i).rob = MmRupdate(hm(i).rob,ge(:,i),th(:,i),xi,[lm;la;le]);
        end
        for i=1:n
            pm_hat(:,i) = SE2(gc_hat(:,i)).t+SE2(gc_hat(:,i)).R*D_z_inflat{i}*z(:,i);
        end
        set(hz,'Xdata',pm_hat(1,:),'YData',pm_hat(2,:));
        hmap = update_region_2d(hmap,nodepoly(polychange(polyidx,2)));
        pause(0.01)
    end
    if video_on
        frame = getframe;
        frame.cdata = imresize(frame.cdata,[480,480]);
        writeVideo(writerObj,frame);
    end
end
if video_on
    close(writerObj);
end

