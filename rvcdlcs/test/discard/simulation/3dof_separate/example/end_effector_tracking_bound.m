% centroid tracking
close all; clear;

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
% initial Fc, Fc_star, Fc_hat
Fc = [[1,1,-1,-1]-3;
    [-1,1,1,-1]-2;
    pi/3,pi/4,pi/6,pi/8];
dotFc = zeros(size(Fc));
Fc_star = [-3;-2;0];
Fc_hat = [Fc_star,Fc(:,2:4)];
dotFc_hat = zeros(size(Fc));
% canvas
figure
subplot(1,1,1);
axis([-5 5 -5 5]); hold on;
hfcstar = SE2_plot(SE2(Fc_star),'r');
for i=1:size(Fc,2)
    hfc(i) = SE2_plot(SE2(Fc(:,i)),'b'); 
    hfchat(i) = SE2_plot(SE2(Fc_hat(:,i)),'g');
end
hold off
% simulation
dt = 0.001;
for t=0:dt:10
    % Fc_star
    dotFc_star = [1;0;0]*(t<2)+[-pi/4*sin(pi/4*(t-2)-pi/2);pi/4*cos(pi/4*(t-2)-pi/2);pi/4]*(t>=2&&t<4)+[0;1;0]*(t>=4&&t<6)+[pi/4*sin(-pi/4*(t-6)+pi);-pi/4*cos(-pi/4*(t-6)+pi);-pi/4]*(t>=6&&t<8)+[1;0;0]*(t>=8);
    % Fc_hat:    dotFc_hat(:) = kron(ones(4,1),dotFc_star)-(Fc_hat(:)-kron(ones(4,1),Fc_star));
    b = [1;0;1;0]; % robot that has access to Fc_star
    M = L+diag(b); gamma = 3;    
    dotFc_hat(:) = kron(M,eye(3))^-1*(-gamma*tanh(kron(M,eye(3))*Fc_hat(:)-kron(b,eye(3))*Fc_star)+kron(b,eye(3))*dotFc_star);
    % Fc
    dotFc_err = dotFc_hat(:)-(Fc(:)-Fc_hat(:)); beta = 3; 
    dotFc(:)=-kron(D,eye(3))*beta*tanh(100*(kron(D',eye(3))*Fc(:)))+dotFc_err;
%     dotFc(:)=-kron(D,eye(3))*beta*sign(kron(D',eye(3))*Fc(:))+dotFc_err;
%     dotFc(:)=-kron(D,eye(3))*(max(dotFerr)+1)*sign(kron(D',eye(3))*Fc(:))+kron(Gamma,eye(3))*kron(ones(4,1),dotFc_star)*4-5*(Fc(:)-kron(Gamma,eye(3))*kron(ones(4,1),Fc_star)*4); 
    % update
    Fc_star = Fc_star + dotFc_star*dt;
    Fc_hat(:) = Fc_hat(:) + dotFc_hat(:)*dt;
    Fc(:) = Fc(:) + dotFc(:)*dt;
    % plot
    set(hfcstar,'Matrix',SE3(SE2(Fc_star)).T);
    for i=1:size(Fc,2)
        set(hfchat(i),'Matrix',SE3(SE2(Fc_hat(:,i))).T);
        set(hfc(i),'Matrix',SE3(SE2(Fc(:,i))).T);
    end
    pause(0.001);
end
