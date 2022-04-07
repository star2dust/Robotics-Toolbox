function [y,ny] = sigmod(x)
% Sigmod function 

y = 1./(exp(-x)+1);
ny = y.*(1-y);
end