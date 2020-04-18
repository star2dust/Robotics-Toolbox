function [ T ] = se3Translation( v,theta )
%se3Translation Exponetial mapping from a pure translation
%
%   T = se3Translation(v,theta) gives the homogeneous transformation from a
%   pure translation. v is the unit vector indicating the translation
%   direction, and theta is the distance of translation.

    T=[eye(3),v*theta;
        0,0,0,1];
end

