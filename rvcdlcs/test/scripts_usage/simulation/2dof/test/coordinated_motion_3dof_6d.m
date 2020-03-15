% coordinated motion 3dof
close all; clear;

%% robot parameters
% mR manipulator (m dof)
m = 3; 
% link length (arm=>la,platform=>lm,end-effector=>le)
la = ones(m,1)*.5; lm = 0.6; le = 0.2;
% get twist
xi = MmRtwist(la);

%% constraint sets
% get q_intsec with best manipulability
negmupower2 = @(q) -sin(q(3)+q(4))^2; % 3dof
lb = [-.5;-.5;-pi/2;-pi/2;.2;0];
ub = [.5;.5;0;0;1.25;.9];
% initial states q = [epcx,epcy,th2,epmcx,epmcy]
th23 = [-pi/4;-pi/4;];
q0 = [0;0;th23;link(2)+link(1)*cos(-th23);link(1)*sin(-th23)];
q_intsec = fmincon(negmupower2,q0,[],[],[],[],lb,ub,@(q) intsectcons(q(1:3),link,q(4:5)));


%% initial states
% type (small=1, big=2)
type = 2;
% desired displacements
epmebar = [0.97,1.09;
        0.68,0.8];
epmemax = [1.5;1.5];
epcl = sqrt(2);
epcr = 0.2;
% a small rotation
phic = - atan2(epmebar(2,type)+epcr,epmebar(1,type)+epcl+epcr);
% initial Fc
Fc = [0;0;phic];
% geometry of the grasp
cpe = [1,-1,-1,1;
    1,1,-1,-1]*epcl/(2*sqrt(2));
cphie = [-3*pi/4,-pi/4,pi/4,3*pi/4];
cFe = [cpe;cphie];
% intial Fe
Fe_SE2 = SE2(Fc')*SE2(cFe');
% for i=1:length(Fe_SE2)
%    Fe_SE2(i).plot;
%    hold on
% end
Fe = SE2toF(Fe_SE2);
% initial eFc
for i=1:size(cFe,2)
    eFc(:,i) = SE2toF(SE2(cFe(:,i)).inv);
end
epc = eFc(1:2,:);
ephic = eFc(3,:);
% initial th23
th23 = kron(ones(1,4),[-0.95,-0.30]');
th = [Fe(3,:)-sum(th23,1);th23];
H = [-ones(1,size(th23,1));eye(size(th23,1))];
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

% range of epc
epc_cen = [epcl;0];
epc_rad = 0.2;
epc_ur = epc_cen+epc_rad;
epc_dl = epc_cen-epc_rad;
% range of cpm [small,big]
epcm_ur = -epmebar-epc_dr;
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
axis([-5 5 -5 5]); hold on;
hobj = patch(Fe(1,:),Fe(2,:),'y');
for i=1:size(Fe,2)
   hrob{i} = MmRplot(Fe(:,i),th(:,i),xi,[lm;la;le],'b'); 
end
hcpmhat = plot(cpmhat(1,:),cpmhat(2,:),'go');
hcpmbar = plot(cpmbar{type}(1,:),cpmbar{type}(2,:),'ro');

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
        [f(:,i),Ji] = mRfkine(H*th23(:,i),la(2:4));
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
        hrob{i} = mRupdate(hrob{i},Fe(:,i),th(:,i),xi,la);
    end
    set(hcpmhat,'XData',cpmhat(1,:),'YData',cpmhat(2,:));
    pause(0.001);
end