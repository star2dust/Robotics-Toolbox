function vs = skew_(v)
%  skew_ Create skew-symmetric matrix
%
%   S = skew(V) is a skew-symmetric matrix formed from V.
%
%   If V (1x1) then S =
%
%             | 0  -v |
%             | v   0 |
%
%   if V (1x2) then S =
%             | -vy vx|
%
%   and if V (1x3) then S =
%
%             |  0  -vz   vy |
%             | vz    0  -vx |
%             |-vy   vx    0 |
%
%
%   Notes::
%   - This is the inverse of the function VEX().
%   - These are the generator matrices for the Lie algebras so(2) and so(3).
%
%   References::
%   - Robotics, Vision & Control: Second Edition, Chap 2,
%     P. Corke, Springer 2016.

for i=1:size(v,1)
    vs = [];
    if isvec(v(i,:),2)
        S = skew([v(i,:),0]);
        vs = [vs;S(3,1:2)];
    elseif isvec(v(i,:),1)||isvec(v(i,:),3)
        vs = [vs;skew(v(i,:))];
    else
        error('unknown size');
    end
end
end