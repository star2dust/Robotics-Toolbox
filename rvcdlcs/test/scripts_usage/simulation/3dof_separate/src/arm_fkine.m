function [f,J] = arm_fkine(th,l)
% forward kinematics and Jacobian
f = [0;0];
for i=1:length(th)
    f = f+l(i)*[cos(sum(th(1:i)));sin(sum(th(1:i)))];
end
J = [];
for i=1:length(th)
    Ji = zeros(2,1);
    for j=length(th):-1:i
        Ji = Ji+l(j)*[-sin(sum(th(1:j)));cos(sum(th(1:j)))];
    end
    J = [J,Ji];
end
end