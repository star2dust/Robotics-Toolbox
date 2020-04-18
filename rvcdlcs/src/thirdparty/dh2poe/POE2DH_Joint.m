function [theta, d, alpha, a]=POE2DH_Joint (xi)
%POE2DH_Joint Conversion from a joint twist to DH parameters
%   
%   [theta, d, alpha, a]=POE2DH_Joint (xi)
%   xi:                     Joint twist, 6 x 1
%   theta, d, alpha, a:     DH parameters

DELTA=10^(-12);

w=xi(1:3);
v=xi(4:6);    
if norm(w)~=0&&abs(norm(w)-1)>DELTA
    error('w should be normalized to a unit vector.')
elseif abs(w'*v)>DELTA
    error('v should be perpendicular to w.')
end
    
if norm(w)~=0 %revolute joint
    alpha=acos(w(3));    
    if abs(w(3)-1)<DELTA || abs(w(3)+1)<DELTA %w(3)==1 or w(3)==-1 
        a = sqrt(v(1)^2+v(2)^2);
        theta = atan2(v(1)/w(3),-v(2)/w(3));
        d = 0;
    else
        %comment these lines between if don't comform to the rule
        %of ensuring a>=0
        asina=-v(3);
        if asina<-DELTA
            alpha=-alpha;
        end
        %comment these lines between if don't comform to the rule
        %of ensuring a>=0
                
        theta = atan2(w(1)/sin(alpha),-w(2)/sin(alpha));
        a = -v(3)/sin(alpha);
        d = (w(1)*v(2)-w(2)*v(1))/(w(1)*w(1)+w(2)*w(2));
    end
else %prismatic joint
    alpha=acos(v(3));
    if abs(v(3)-1)<DELTA || abs(v(3)+1)<DELTA  %v(3)==1 or v(3)==-1
        theta = 0;
    else
        theta = atan2(v(1)/sin(alpha),-v(2)/sin(alpha));
    end
    a=0;
    d=0;
end
end