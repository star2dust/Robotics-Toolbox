% coordinated motion with displacement determined by cpm_bar
close all; clear;

% write video
isvideowritten = false;

%% robot parameters
% mR manipulator (m dof)
m = 2; 
% link length (arm=>la,platform=>lm,end-effector=>le)
la = ones(m,1)*.5; lm = 0.6; le = 0.2;
% get twist
xi = MmRtwist(la);

%% geometry of the grasp
% initial Fc
Fc = [0;0;0];
% geometry of the grasp
epcl = 1;
cpe_bar = [1,-1,-1,1;
    1,1,-1,-1]*epcl/sqrt(2);
cphie_bar = [-3*pi/4,-pi/4,pi/4,3*pi/4];
cFe_bar = [cpe_bar;cphie_bar];

%% bounded constraints
% epc range
epc_cen = [epcl;0];
epc_rad = 0.2;
epc_ur = epc_cen+epc_rad;
epc_dl = epc_cen-epc_rad;
epc_range = [epc_ur(1),epc_dl(1),epc_dl(1),epc_ur(1),epc_ur(1);
    epc_ur(2),epc_ur(2),epc_dl(2),epc_dl(2),epc_ur(2)];
for i=1:size(cFe_bar,2)
    cpe_range{i} = -rot2(cphie_bar(i))*epc_range;
end
% th2 range
th2_ur = -asin(0.2);
th2_dl = -pi/2;
% epcm range
epcm_ur = -epc_dl-la(2)+0.2;
epcm_dl = -epc_ur-sum(la)-.5;
epcm_range = [epcm_ur(1),epcm_dl(1),epcm_dl(1),epcm_ur(1),epcm_ur(1);
    epcm_ur(2),epcm_ur(2),epcm_dl(2),epcm_dl(2),epcm_ur(2)];
for i=1:size(cFe_bar,2)
    cpm_range{i} = -rot2(cphie_bar(i))*epcm_range;
end

%% intersection constraints
% get q_intsec with best manipulability
negmupower2 = @(q) -sin(q(3))^2; % 2dof
lb = [epc_dl;th2_dl;epcm_dl];
ub = [epc_ur;th2_ur;epcm_ur];
% intersection of hyperplanes
th2 = -pi/4;
H = [-ones(1,size(th2,1));eye(size(th2,1))];
q0 = [epc_cen;th2;-epc_cen-mRfkine(H*th2,la);];
q_intsec = fmincon(negmupower2,q0,[],[],[],[],lb,ub,@(q) intsectcons(q,la));
mu_bar = la(1)*la(2)*abs(sin(q_intsec(3)));
% modify joint space to convex space => normal vector of epc+ef(q)+cpm=0
nx = [1;0;sin(-q_intsec(3))*la(1);1;0];
ny = [0;1;-cos(-q_intsec(3))*la(1);0;1];
% tangent plane
consA = [nx,ny]'; consb = consA*q_intsec;
% initial q = [epcx,epcy,th2,epcmx,epcmy]
q = lsqlin(eye(length(q0)),q0,consA,consb,[],[],lb,ub);
q = kron(ones(1,size(cFe_bar,2)),q);

%% initial robot states
% initial eFc
epc = q(1:2,:);
ephic = -cphie_bar;
eFc = [epc;ephic];
% intial Fe
for i=1:size(eFc,2)
    Fe_SE2(i) = SE2(Fc)*SE2(eFc(:,i)).inv;
    Fe(:,i) = SE2toPose(Fe_SE2(i));
end
% initial th
th2 = q(3,:);
th = [Fe(3,:)-sum(th2,1);th2];
% initial epcm
epcm_hat = q(end-1:end,:);
for i=1:size(epcm_hat,2)
    cpm_hat(:,i) = rot2(cphie_bar(i))*epcm_hat(:,i);
end

% desired displacements
cpm_hat0 = cpm_hat;
cpm_bar = rot2(pi/20)*cpm_hat0+1;

% write avi
if isvideowritten
    videoname = 'coordinated_motion3';
    writerObj = VideoWriter(['../result/',videoname]);
    open(writerObj);
end

%% initial canvas
% plot constraint sets
figure
subplot(1,1,1) 
axis([-5 5 -5 5]); hold on;
% static plot
hcpm_bar = plot(cpm_bar(1,:),cpm_bar(2,:),'g*');
for i=1:size(cFe_bar,2)
    plot(cpe_range{i}(1,:),cpe_range{i}(2,:),'r-.');
    hcpm_range(i) = plot(cpm_range{i}(1,:),cpm_range{i}(2,:),'g');
end
% plot object and robots
hobj = patch('XData',Fe(1,:),'YData',Fe(2,:),'FaceColor','y','FaceAlpha',0.5);
for i=1:size(Fe,2)
   hrob{i} = MmRplot(Fe(:,i),th(:,i),xi,[lm;la;le],'b'); 
end
hcpm_hat = plot(cpm_hat(1,:),cpm_hat(2,:),'go');
hold off


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

%% simulation
dot_q_ex = zeros(size(q));
dot_epc_ex = zeros(size(epc));
dot_th2_ex = zeros(size(th2));
dot_epcm_hat_ex = zeros(size(epcm_hat));
lambda = zeros(size(epc));
eta = zeros(size(epcm_hat));
dt = 0.01;
for t=0:dt:6
    if t==2
        % epcm range change
        cpm_hat0 = cpm_hat0*0.6;
        cpm_bar = rot2(pi/20)*cpm_hat0+1;
        epcm_ur = -epc_dl-la(2)+0.2;
        epcm_dl = (-epc_ur-sum(la))*0.6-.5;
        epcm_range = [epcm_ur(1),epcm_dl(1),epcm_dl(1),epcm_ur(1),epcm_ur(1);
            epcm_ur(2),epcm_ur(2),epcm_dl(2),epcm_dl(2),epcm_ur(2)];
        for i=1:size(cFe_bar,2)
            cpm_range{i} = -rot2(cphie_bar(i))*epcm_range;
        end
        lb = [epc_dl;th2_dl;epcm_dl];
        ub = [epc_ur;th2_ur;epcm_ur];
    end
    % calculate forward kinematics and Jacobian
    jh = []; eRc = [];
    for i=1:size(epc,2)
        [f(:,i),Ji] = mRfkine(H*th2(:,i),la);
        jh=blkdiag(jh,Ji*H);
        eRc = blkdiag(eRc,rot2(ephic(i)));
    end
    % calculate manipulability
    for i=1:size(th2,2)
        [mu(:,i),munabla(:,i)] = mRmanipulability(H*th2(:,i),la);
    end
%     mu = abs(sin(sum(th2,1)))'; 
%     munabla = [];
%     for i=1:size(th2,2)
%         munabla = [munabla;tanh(100*sin(sum(th2(:,i),1)))*cos(sum(th2(:,i),1))*ones(size(th2(:,i),1),1)];
% %         munabla = [munabla;sign(sin(sum(th2(:,i),1)))*cos(sum(th2(:,i),1))*ones(size(th2(:,i),1),1)];
%     end
    % calculate err_d and err_mu
    k1 = 1; k2 = 1;
    err_d = k1*(epc(:)+f(:)+epcm_hat(:));
    err_mu = k2*(mu(:)-ones(size(mu,2),1)*mu_bar);
    % calculate inside dot_q_ex
    dot_epc_ex(:) = -err_d-eRc*lambda(:);
    dot_th2_ex(:) = -jh'*err_d-kron(diag(err_mu),eye(size(th2,1)))*munabla(:);
    dot_epcm_hat_ex(:) = -err_d-eRc*kron(L,eye(2))*eta(:);
    dot_q_ex = [dot_epc_ex;dot_th2_ex;dot_epcm_hat_ex];
    % calculate dot_q
    for i=1:size(dot_q_ex,2)
        dot_q(:,i) = lsqlin(eye(size(dot_q_ex,1)),q(:,i)+dot_q_ex(:,i),consA,consb,[],[],lb,ub)-q(:,i);
    end
    dot_epc = dot_q(1:2,:);
    dot_th2 = dot_q(3:end-2,:);
    dot_epcm_hat = dot_q(end-1:end,:);
    dot_eta = kron(L,eye(2))*(eRc'*epcm_hat(:)+eRc'*dot_epcm_hat(:)-cpm_bar(:));
    sgn_lambda = zeros(size(epc));
    for i=1:size(epc,2)
        for j=1:size(epc,2)
            sgn_lambda(:,i) = sgn_lambda(:,i) + A(i,j)*tanh(100*(lambda(:,i)-lambda(:,j)));
%             sgn_lambda(:,i) = sgn_lambda(:,i) + A(i,j)*sign(lambda(:,i)-lambda(:,j));
        end
    end
    alpha = 2;
    dot_lambda = eRc'*(epc(:)+dot_epc(:))-alpha*sgn_lambda(:);
    % update
    q(:) = q(:)+dot_q(:)*dt;
    lambda(:) = lambda(:)+dot_lambda(:)*dt;
    eta(:) = eta(:)+dot_eta(:)*dt;
    % get epc th2 epcm_hat from q
    epc = q(1:2,:);
    th2 = q(3:end-2,:);
    epcm_hat = q(end-1:end,:);
    % plot
    set(hcpm_bar,'XData',cpm_bar(1,:),'YData',cpm_bar(2,:));
    for i=1:size(cFe_bar,2)
        set(hcpm_range(i),'XData',cpm_range{i}(1,:),'YData',cpm_range{i}(2,:));
    end
    eFc = [epc;ephic];
    eFc_SE2 = SE2(eFc');
    for i=1:length(eFc_SE2)
       Fe_SE2(i) = SE2(Fc)*eFc_SE2(i).inv;
       Fe(:,i) = SE2toPose(Fe_SE2(i));
    end
    set(hobj,'XData',Fe(1,:),'YData',Fe(2,:));
    th = [Fe(3,:)-sum(th2,1);th2];
    for i=1:size(Fe,2)
        hrob{i} = MmRupdate(hrob{i},Fe(:,i),th(:,i),xi,[lm;la;le]);
    end
    for i=1:size(epcm_hat,2)
        cpm_hat(:,i) = rot2(cphie_bar(i))*epcm_hat(:,i);
    end
    set(hcpm_hat,'XData',cpm_hat(1,:),'YData',cpm_hat(2,:));
    pause(0.001);
    if isvideowritten
        frame = getframe;
        frame.cdata = imresize(frame.cdata,[480,480]);
        writeVideo(writerObj,frame);
    end
end
if isvideowritten
    close(writerObj);
end