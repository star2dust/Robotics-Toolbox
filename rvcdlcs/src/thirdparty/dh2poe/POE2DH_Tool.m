function [theta1, d1, alpha1, a1, theta2, d2] = POE2DH_Tool (g)
%POE2DH_Tool Conversion from an initial twist to DH parameters
%   
%   g:      Homogeneous transformation representing the intitial state of the
%   robot, 4 x 4
%   theta1, d1, alpha1, a1, theta2, d2:   Associated DH parameters

DELTA=10^(-12);
R = g(1:3,1:3);
eul = ROTM2EUL(R, 'ZXZ');
theta1 = eul(1);
alpha1 = eul(2);
theta2 = eul(3);
b = g(1:3,4);
if abs(sin(alpha1))<DELTA %alpha1=0 or alpha1=pi
    a1=sqrt(b(1)^2+b(2)^2);
    if abs(b(1))<DELTA && abs(b(2))<DELTA %b(1)=0 & b(2)=0
        theta1 = 0;
    else
        theta1 = atan2(b(2),b(1));
    end
    temp = rotx(-alpha1)*rotz(-theta1)*R;
    if abs(temp(2,1))<DELTA && abs(temp(2,2))<DELTA
        theta2 = 0;
    else
        theta2 = atan2(temp(2,1),temp(2,2));
    end
    d1=b(3);
    d2=0;
else
    A = [0  cos(theta1) sin(theta1)*sin(alpha1)
        0   sin(theta1) -cos(theta1)*sin(alpha1)
        1   0           cos(alpha1)];
    x=A\b;
    d1=x(1);
    a1=x(2);
    d2=x(3);    
end
end