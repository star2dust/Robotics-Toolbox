function g = fkPOE (POE_par, q)
% fkPOE forward kinematics using POE formula
%
%   g = fkPOE (POE_par, q)
%   POE_par:    POE parameters, 6 x n
%   q:          Joint variables, n-1 x 1
%   g:          homogeneous transformation, 4 x 4

n=size(POE_par,2);

g=eye(4);

for i=1:n-1
    g=g*se3Exp(POE_par(:,i)*q(i));
end

g=g*se3Exp(POE_par(:,n));

end