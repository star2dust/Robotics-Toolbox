function [c,ceq]=intsectcons(q,l)
import mpr.*
% parameters
epc = q(1:2);
th2 = q(3:end-2);
epmc = -q(end-1:end);
% change to end-effector frame
H = [-ones(1,size(th2,1));eye(size(th2,1))];
th = H*th2;
% result
c = [];
ceq = epc+fkine(th,l)-epmc;
end