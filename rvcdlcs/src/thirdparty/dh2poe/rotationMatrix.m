function [ R ] = rotationMatrix( w,theta )
%rotationMatrix Rotation from axis and angle
%
%   R = rotationMatrix(w,theta) gives the rotation matrix about a unit axis
%   w for an angle of theta.
%   w:      Unit rotation axis, 3 x 1
%   theta:  rotation algle
%   R:      Rotation matrix, 3 x 3
R=eye(3)+skew(w)*sin(theta)+skew(w)*skew(w)*(1-cos(theta));

end

