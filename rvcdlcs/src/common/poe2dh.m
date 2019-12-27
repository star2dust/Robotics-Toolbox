function dh_mat = poe2dh(xi_mat)
import SE3.*

n=size(xi_mat,2)-1; %number of joints 

dh_mat = zeros(n+2, 4);

xi = xi_mat(:,1);

g = eye(4);

for i=1:n

[theta, d, alpha, a]=POE2DH_Joint ([xi(4:6);xi(1:3)]); % change position of w and v

dh_mat(i,:) = [theta, d, alpha, a];

g = g*stdDH (dh_mat(i,:));

if i~=n
    xi = Adg(inv(g))*xi_mat(:,i+1);
end
    
end

[theta1, d1, alpha1, a1, theta2, d2] = POE2DH_Tool (g\expm(hatwedge(xi_mat(:,n+1))));

dh_mat(n+1,:) = [theta1, d1, alpha1, a1];
dh_mat(1,:) = [];
end