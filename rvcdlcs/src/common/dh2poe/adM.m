function [ ad ] = adM( M )
%adM adjoint transformation
%
%   [ ad ] = adM( M )
%   M:  Homogeneous transformation, 4 x 4
%   ad: Adjoint transformation, 6 x 6

R=M(1:3,1:3);
p=M(1:3,4);
ad=zeros(6,6);
ad(1:3,1:3)=R;
ad(4:6,1:3)=skew(p)*R;
ad(4:6,4:6)=R;
end