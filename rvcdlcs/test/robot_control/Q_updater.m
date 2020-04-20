function Qnow = Q_updater(robot,lidar,qrob,qf,qf_next,pfd,Qmax,val_max,var_max)


% qrob split
[s,pfe,thfe,qae,qa,qb,qe,qc] = qrob_split(robot,qrob,qf);

% s = qrob(:,1); qfe = qrob(:,2:7); qae = qrob(:,8:end);
% qe = toqrpy(SE3.qrpy(qf).*SE3.qrpy(qfe));
% qa = [qe(:,end)-sum(qae,2),qae];
% qb = robot.bkine(qa,qe);

% formation
thfd = cart2pol(pfd(:,1),pfd(:,2));
thd = qf(:,3)+thfd;
% lidar detect
hold on
for j=1:length(robot)
    s_max = Qmax.qlim{j}(2,1); 
    pfdl = s_max*pfd(j,:)+val_max(j,:);
    pfdr = s_max*pfd(j,:)+var_max(j,:);
    Vfdl = [zeros(1,2);s_max*pfd(j,:);pfdl;zeros(1,2)];
    Vfdr = [zeros(1,2);s_max*pfd(j,:);pfdr;zeros(1,2)];
    Vdc = lidar(j).detect([qb(j,1:2),thd(j)],1);
    % * equals to .* for SE2 
%     Vdl = (SE2(qf(j,:))*Vfdl')';
%     Vdr = (SE2(qf(j,:))*Vfdr')';
%     plot(Vdl(:,1),Vdl(:,2),'b');
%     plot(Vdr(:,1),Vdr(:,2),'c');
    % Vdc in qf frame
    Vfdcl = []; Vfdcr = [];
    for i=1:length(Vdc)
        Vfdcdt = [(SE2(qf(j,:)).inv*Vdc{i}')';
            (SE2(qf_next(j,:)).inv*Vdc{i}')'];
%         Vdcdt = [h2e(SE2(qf(j,:)).inv.T*e2h(Vdc{i}'))';
%             h2e(SE2(qf_next(j,:)).inv.T*e2h(Vdc{i}'))'];
        Vfdc = Vfdcdt(convhull_(Vfdcdt),:);
%         Vdcdt = (SE2(qf(j,:))*Vfdc')';
%         plot(Vdc{i}(:,1),Vdc{i}(:,2),'r-');
%         plot(Vdcdt(:,1),Vdcdt(:,2),'r:');
        % left and right
        Vfdcl = [Vfdcl;polyxpoly_(Vfdc,Vfdl)];
        Vfdcr = [Vfdcr;polyxpoly_(Vfdc,Vfdr)];    
    end
    Vdcl = (SE2(qf(j,:))*Vfdcl(convhull_(Vfdcl),:)')';
    Vdcr = (SE2(qf(j,:))*Vfdcr(convhull_(Vfdcr),:)')';
%     plot(Vdcl(:,1),Vdcl(:,2),'b','linewidth',2);
%     plot(Vdcr(:,1),Vdcr(:,2),'c','linewidth',2);
    [Al{j},bl{j},Aleq{j},bleq{j}] = polycons(Vfdcl);
    [Ar{j},br{j},Areq{j},breq{j}] = polycons(Vfdcr);
end
% obstacle divide
Qnow = Qmax;
for i=1:length(robot)
    if isempty(Al{i})&&isempty(Aleq{i})
        sl = s_max; 
    else
        % left
        vals = skew_(val_max(i,:));
        fl = vals'/(vals*pfd(i,:)');
        [xl,sl] = linprog(fl,Al{i},bl{i},Aleq{i},bleq{i});   
    end
    if isempty(Ar{i})&&isempty(Areq{i})
        sr = s_max; 
    else
        % right
        vars = skew_(var_max(i,:));
        fr = vars'/(vars*pfd(i,:)');
        [xr,sr] = linprog(fr,Ar{i},br{i},Areq{i},breq{i});   
    end
    % Qnow
    sm = Qnow.qlim{i}(2,1);
    Qnow.qlim{i}(2,1) = min([sm,sr,sl]);
    if diff(Qnow.qlim{i}(:,1))<0
       warning('infeasible limitation') 
    end
%     pfdr_now = min([sm,sr,sl])*pfd(j,:)+min([sm,sr,sl])*var_max(j,:);
%     Vfdr_now = [zeros(1,2);min([sm,sr,sl])*pfd(j,:);pfdr_now;zeros(1,2)];
%     Vdr_now = (SE2(qf(j,:))*Vfdr_now')';
%     plot(Vdr_now(:,1),Vdr_now(:,2),'c');
end
end