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
% initial Fc_star
Fc_star = [-3;-2;0];
% initial Fb
csFb = [0;0;phib];
% geometry of the grasp
bpe = [1,0,-1,0;
    0,1,0,-1]*epcl;
bphie = [-pi,-pi/2,0,pi/2];
bFe = [bpe;bphie];
% intial Fe
Fe_SE2 = SE2(Fc_star')*SE2(csFb')*SE2(bFe');
Fe = SE2toF(Fe_SE2);
N = size(Fe,2);
% initial Fc
Fc = sum(Fe,2)/N;
cFcs_SE2 = SE2(Fc).inv*SE2(Fc_star);
% initial eFc
cFe_SE2 = cFcs_SE2*SE2(csFb')*SE2(bFe');
for i=1:length(cFe_SE2)
    eFc(:,i) = SE2toF(cFe_SE2(i).inv);
end
epc = eFc(1:2,:);
ephic = eFc(3,:);
% initial Fc_hat
Fc_hat = kron(ones(1,N),Fc);
Fcs_hat = [Fc_star,Fe(:,2),Fc_star,Fe(:,4)];
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
for i=1:N
   hrob{i} = robot_plot(Fe(:,i),th(:,i),xi,l,'b'); 
end
% hcpmhat = plot(cpmhat(1,:),cpmhat(2,:),'go');
% for i=1:2
%     hcpmbar = plot(cpmbar{i}(1,:),cpmbar{i}(2,:),'ro');
% end
for i=1:size(Fc_hat,2)
    hfchat(i) = SE2_plot(SE2(Fc_hat(:,i)),'b'); 
    hfcshat(i) = SE2_plot(SE2(Fcs_hat(:,i)),'g-.');
end
hfcstar = SE2_plot(SE2(Fc_star),'r');
hfc = SE2_plot(SE2(Fc),'m-.');

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
dotFcs_hat = zeros(size(Fcs_hat));
lambda = zeros(size(epc));
eta = zeros(size(cpmhat));
dt = 0.001; tf = 10; count = 0;
for t=0:dt:tf
    % calculate forward kinematics and Jacobian
    jh = []; eRc = [];
    for i=1:size(epc,2)
        [f(:,i),Ji] = arm_fkine(H*th23(:,i),l(2:4));
        jh=blkdiag(jh,Ji*H);
        eRc = blkdiag(eRc,rot2(ephic(i)));
    end
    % calculate manipulability
    mu = cos(sum(th23,1)+pi/2)';
    munabla = [];
    for i=1:size(th23,2)
        munabla = [munabla;-sin(sum(th23(:,i))+pi/2)*ones(size(th23,1),1)];
    end
    % calculate err_d and err_mu
    k1 = 1; k2 = 0;
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
        for j=1:size(epc,2) % sign(x) is replaced by tanh(100*x) 
            sgnlambda(:,i) = sgnlambda(:,i) + A(i,j)*tanh(100*(lambda(:,i)-lambda(:,j)));
        end
    end
    alpha = 2;
    dotlambda = eRc'*(epc(:)+dotepc(:))-alpha*sgnlambda(:);
    % Fc_star
    dotFc_star = [1;0;0]*(t<tf/5)+[-pi/4*sin(pi/4*(t-tf/5)-pi/2);pi/4*cos(pi/4*(t-tf/5)-pi/2);pi/4]*(t>=tf/5&&t<2*tf/5)+[0;1;0]*(t>=2*tf/5&&t<3*tf/5)+[pi/4*sin(-pi/4*(t-3*tf/5)+pi);-pi/4*cos(-pi/4*(t-3*tf/5)+pi);-pi/4]*(t>=3*tf/5&&t<4*tf/5)+[1;0;0]*(t>=4*tf/5);
    % Fcs_hat:    dotFc_hat(:) = kron(ones(4,1),dotFc_star)-(Fc_hat(:)-kron(ones(4,1),Fc_star));
    b = [1;0;1;0]; % robot that has access to Fc_star
    M = L+diag(b); gamma = 5;    
    dotFcs_hat(:) = kron(M,eye(3))^-1*(-gamma*tanh(kron(M,eye(3))*Fcs_hat(:)-kron(b,eye(3))*Fc_star)+kron(b,eye(3))*dotFc_star);
    % Fe
    eFc = [epc;ephic]; Re = []; 
    for i=1:size(eFc,2)
       Re = blkdiag(Re,SE2([0,0,Fe(3,i)]).T);
    end
%     dotFcs_err = kron(b*N/2,dotFc_star)-(Fe(:)-kron(b*N/2,Fc_star)); beta = 3; 
    dotFcs_err = dotFcs_hat(:)-(Fe(:)-Fcs_hat(:)); beta = 3; 
    dotFe(:)=-kron(D,eye(3))*beta*tanh(100*(kron(D',eye(3))*(Fe(:)+Re*eFc(:))))+dotFcs_err;
    Fc_hat(:) = Fe(:)+Re*eFc(:);
    Fc = sum(Fe,2)/N;
    % update
    epc(:) = epc(:)+dotepc(:)*dt;
    th23(:) = th23(:)+dotth23(:)*dt;
    cpmhat(:) = cpmhat(:)+dotcpmhat(:)*dt;
    lambda(:) = lambda(:)+dotlambda(:)*dt;
    eta(:) = eta(:)+doteta(:)*dt;
    Fc_star(:) = Fc_star(:)+dotFc_star(:)*dt;
    Fcs_hat(:) = Fcs_hat(:)+dotFcs_hat(:)*dt; 
    Fe(:) = Fe(:)+dotFe(:)*dt;
    % plot
    set(hobj,'XData',Fe(1,:),'YData',Fe(2,:));
    th = [Fe(3,:)-sum(th23,1);th23];
    for i=1:N
        hrob{i} = robot_update(hrob{i},Fe(:,i),th(:,i),xi,l);
    end
%     set(hcpmhat,'XData',cpmhat(1,:),'YData',cpmhat(2,:));
    set(hfcstar,'Matrix',SE3(SE2(Fc_star)).T);
    set(hfc,'Matrix',SE3(SE2(Fc)).T);
    for i=1:size(Fc_hat,2)
        set(hfcshat(i),'Matrix',SE3(SE2(Fcs_hat(:,i))).T);
        set(hfchat(i),'Matrix',SE3(SE2(Fc_hat(:,i))).T);
    end
    count = count+1;
    if count>20
        pause(0.001);
        count=0;
    end
end