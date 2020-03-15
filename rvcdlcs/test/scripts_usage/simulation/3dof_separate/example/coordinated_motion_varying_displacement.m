% coordinated motion with varying displacement determined by cpmbar
close all; clear;

%% robot parameters
% link length
l0 = 0.4; l1 = 0.5; l2 = 0.5; l3 = 0.5; l4 = 0.1;
l = [l0,l1,l2,l3,l4];
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
xi = {xi1,xi2,xi3,xi4,xi5,xi6};

%% initial states
% type (small=1, big=2)
type = 2;
% a small rotation
epmebar = [0.97,1.09;
        0.68,0.8];
epmemax = [1.5;1.5];
epcl = 0.5;
epcr = 0.2;
phib = pi/4 - atan2(epmebar(2,type)+epcr,epmebar(1,type)+epcl+epcr);
% initial Fc
Fc = [0;0;0];
% initial Fb
cFb = [0;0;phib];
% geometry of the grasp
bpe = [1,0,-1,0;
    0,1,0,-1]*epcl;
bphie = [-pi,-pi/2,0,pi/2];
bFe = [bpe;bphie];
% intial Fe
Fe_SE2 = SE2(Fc')*SE2(cFb')*SE2(bFe');
Fe = SE2toF(Fe_SE2);
% initial eFc
cFe_SE2 = SE2(cFb')*SE2(bFe');
for i=1:length(cFe_SE2)
    eFc(:,i) = SE2toF(cFe_SE2(i).inv);
end
epc = eFc(1:2,:);
ephic = eFc(3,:);
% initial th23
th23 = kron(ones(1,4),[-0.95,-0.30]');
th = [Fe(3,:)-sum(th23,1);th23];
H = [-ones(1,size(th23,1));eye(size(th23,1))];
% range of epc [small,big]
epc_cen = [epcl;0];
epc_rad = 0.2;
epc_ur = [epc_cen-epc_rad/2,epc_cen+epc_rad];
epc_dl = epc_cen-epc_rad;
% range of cpm [small,big]
epcm_ur = -epmebar-epc_ur;
epcm_dl = -epmemax-epc_ur;
% range of th [small,big] 
th_intsec = [-1.1954,-0.2186;
         -0.3683,-0.8260];
% intial displacement
for j=1:size(epmebar,2)
    epcmhat{j} = kron(ones(1,4),-epmebar(:,j)-epc_ur(:,j));
    % initial cpm {small,big}
    for i=1:size(epcmhat{j},2)
        if j==type
            cpmhat(:,i) = cFe_SE2(i).R*epcm_dl(:,j);
        end
        cpmbar{j}(:,i) = cFe_SE2(i).R*epcmhat{j}(:,i);
    end
end


%% initial canvas
figure
subplot(1,1,1) 
axis([-3 3 -3 3]); hold on;
hobj = patch(Fe(1,:),Fe(2,:),'y');
for i=1:size(Fe,2)
   hrob{i} = robot_plot(Fe(:,i),th(:,i),xi,l,'b'); 
end
hcpmhat = plot(cpmhat(1,:),cpmhat(2,:),'go');
for i=1:2
    hcpmbar = plot(cpmbar{i}(1,:),cpmbar{i}(2,:),'ro');
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

%% simulation
err_d = zeros(size(epc));
err_mu = zeros(size(ephic));
dotepcex = zeros(size(epc));
dotth23ex = zeros(size(th23));
dotcpmhatex = zeros(size(cpmhat));
dotepc = zeros(size(epc));
dotth23 = zeros(size(th23));
dotcpmhat = zeros(size(cpmhat));
lambda = zeros(size(epc));
eta = zeros(size(cpmhat));
dt = 0.01;
for t=0:dt:30
    % calculate forward kinematics and Jacobian
    jh = []; eRc = [];
    for i=1:size(epc,2)
        [f(:,i),Ji] = arm_fkine(H*th23(:,i),l(2:4));
        jh=blkdiag(jh,Ji*H);
        eRc = blkdiag(eRc,rot2(ephic(i)));
    end
    % calculate manipulability
    mu = cos(sum(th23,1))';
    munabla = [];
    for i=1:size(th23,2)
        munabla = [munabla;-sin(sum(th23(:,i)))*ones(size(th23,1),1)];
    end
    % calculate err_d and err_mu
    k1 = 1; k2 = 1;
    err_d(:) = k1*(epc(:)+f(:)+eRc*cpmhat(:));
    err_mu(:) = k2*(mu-1);
    % calculate inside
    dotepcex(:) = -err_d(:)-eRc*lambda(:);
    dotth23ex(:) = -jh'*err_d(:)-kron(diag(err_mu(:)),eye(2))*munabla;
    dotcpmhatex(:) = -eRc'*err_d(:)-kron(L,eye(2))*eta(:);
    % calculate dot
    for i=1:size(epc,2)
        dotepc(:,i) = pt_proj(epc(:,i)+dotepcex(:,i),epc_ur(:,type),epc_dl)-epc(:,i);
        dotth23(:,i) = th_proj(th23(:,i)+dotth23ex(:,i),th_intsec(:,type))-th23(:,i);
        dotcpmhat(:,i) = rot2(ephic(i))'*pt_proj(rot2(ephic(i))*(cpmhat(:,i)+dotcpmhatex(:,i)),epcm_ur(:,type),epcm_dl(:,type))-cpmhat(:,i);
    end
    doteta = kron(L,eye(2))*(cpmhat(:)+dotcpmhat(:)-cpmbar{type}(:));
    sgnlambda = zeros(size(epc));
    for i=1:size(epc,2)
        for j=1:size(epc,2)
            sgnlambda(:,i) = sgnlambda(:,i) + A(i,j)*sign(lambda(:,i)-lambda(:,j));
        end
    end
    alpha = 2;
    dotlambda = eRc'*(epc(:)+dotepc(:))-alpha*sgnlambda(:);
    % update
    epc(:) = epc(:)+dotepc(:)*dt;
    th23(:) = th23(:)+dotth23(:)*dt;
    cpmhat(:) = cpmhat(:)+dotcpmhat(:)*dt;
    lambda(:) = lambda(:)+dotlambda(:)*dt;
    eta(:) = eta(:)+doteta(:)*dt;
    % plot
    eFc = [epc;ephic];
    eFc_SE2 = SE2(eFc');
    for i=1:length(eFc_SE2)
       Fe_SE2(i) = SE2(Fc)*eFc_SE2(i).inv;
       Fe(:,i) = SE2toF(Fe_SE2(i));
    end
    set(hobj,'XData',Fe(1,:),'YData',Fe(2,:));
    th = [Fe(3,:)-sum(th23,1);th23];
    for i=1:size(Fe,2)
        hrob{i} = robot_update(hrob{i},Fe(:,i),th(:,i),xi,l);
    end
    set(hcpmhat,'XData',cpmhat(1,:),'YData',cpmhat(2,:));
    pause(0.001);
end