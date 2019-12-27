function [ T ] = se3Rotation( w,v,theta )
%se3Rotation Exponential mapping from se3 to SE3 for a non-pure-translation
%motion
%   
%   T = se3Rotation(w,v,theta) gives the homogeneous transformation from a
%   twist. 
%   xi = [w;v], w is a unit vector describing the rotation axis of the twist. 
%   T = exp(xi*theta)

R=rotationMatrix(w,theta);
p=(theta*eye(3)+(1-cos(theta))*skew(w)+(theta-sin(theta))*skew(w)*skew(w))*v;
T=[R,p;
    0,0,0,1];
end