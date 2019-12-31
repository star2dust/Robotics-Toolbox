function [dh_mat, H_base, H_tool] = poe2dh(xi_mat)
% Conversion from POE parameters to DH parameters
% dh_mat: m x 4 => [link01; link12; ...; link{m-1}m]
% H_base => stdDH(base2link0)

n=size(xi_mat,2)-1; %number of joints 

dh_mat = zeros(n+1, 4);

xi = xi_mat(:,1);

g = eye(4);

for i=1:n

[theta, d, alpha, a]=POE2DH_Joint ([xi(4:6);xi(1:3)]); % change position of w and v

dh_mat(i,:) = [theta, d, a, alpha];

g = g*stdDH (dh_mat(i,:));

if i~=n
    xi = Adg(inv(g))*xi_mat(:,i+1);
end
    
end

[theta1, d1, alpha1, a1, theta2, d2] = POE2DH_Tool (g\expm(wedge(xi_mat(:,n+1))));

dh_mat(n+1,:) = [theta1, d1, a1, alpha1];

H_base = stdDH(dh_mat(1,:));
dh_mat(1,:) = [];
H_tool = trotz(theta2)*transl([0,0,d2]);

eps = 10^-12;
dh_mat(abs(dh_mat)<eps) = 0;
end