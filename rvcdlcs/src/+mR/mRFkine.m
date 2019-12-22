function [f,J] = mRFkine(th,l)
% forward kinematics wrt platform frame (p_me)
p_me = [0;0];
for i=1:length(th)
    p_me = p_me+l(i)*[cos(sum(th(1:i)));sin(sum(th(1:i)))];
end
f = p_me;
% Jacobian wrt platform frame
J = [];
for i=1:length(th)
    Ji = zeros(2,1);
    for j=length(th):-1:i
        Ji = Ji+l(j)*[-sin(sum(th(1:j)));cos(sum(th(1:j)))];
    end
    J = [J,Ji];
end
end