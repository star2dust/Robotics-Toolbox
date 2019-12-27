function DH_par = POE2DH (POE_par)
%POE2DH Conversion from POE parameters to DH parameters
%
%   DH_par = POE2DH (POE_par)
%   POE_par:    POE parameters, 6 x n+1
%   DH_par:     DH parameters,  n+2 x 4  

n=size(POE_par,2)-1; %number of joints 

DH_par = zeros(n+2, 4);

xi = POE_par(:,1);

g = eye(4);

for i=1:n

[theta, d, alpha, a]=POE2DH_Joint (xi);

DH_par(i,:) = [theta, d, alpha, a];

g = g*DH (DH_par(i,:),'std');

if i~=n
    xi = adM(inv(g))*POE_par(:,i+1);
end
    
end

[theta1, d1, alpha1, a1, theta2, d2] = POE2DH_Tool (g\se3Exp(POE_par(:,n+1)));

DH_par(n+1,:) = [theta1, d1, alpha1, a1];

DH_par(n+2,:) = [theta2, d2, NaN, NaN];

end