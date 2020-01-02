function [dh_mat, H_tool, sigma] = poe2dh(xi_mat, q_base)
% Conversion from POE parameters to DH parameters
% dh_mat: m x 4 => [link01; link12; ...; link{m-1}m]
% q_base: theta (R) or d (P) => H_base = stdDH(base2link0)

if nargin<2
   q_base = 0; 
end

n=size(xi_mat,2)-1; %number of joints 

dh_mat = zeros(n+1,4);

sigma = zeros(n+1,1);

xi = xi_mat(:,1);

if xi(6) == 0
    g = trotz(q_base);
else
    g = transl([0,0,q_base]);
end

for i=1:n
    
    % An optional fifth column sigma indicate revolute (sigma=0) or prismatic (sigma=1).
    if xi(6)==0
        sigma(i,1) = 1;
    else
        sigma(i,1) = 0;
    end
    
    [theta, d, alpha, a]=POE2DH_Joint ([xi(4:6);xi(1:3)]); % change position of w and v
    
    dh_mat(i,:) = [theta, d, a, alpha];
    
    g = g*stdDH (dh_mat(i,:));
    
    if i~=n
        xi = Adg(inv(g))*xi_mat(:,i+1);
    end

end

% tool
[theta1, d1, alpha1, a1, theta2, d2] = POE2DH_Tool (g\expm(wedge(xi_mat(:,n+1))));
dh_mat(n+1,:) = [theta1, d1, a1, alpha1];
H_tool = trotz(theta2)*transl([0,0,d2]);

% base
dh_mat(1,:) = [];
if sigma(1)==0
    dh_mat(1,2) = dh_mat(1,2)+q_base;
else
    dh_mat(1,1) = dh_mat(1,1)+q_base;
end
sigma(end) = []; 

eps = 10^-12;
dh_mat(abs(dh_mat)<eps) = 0;
end