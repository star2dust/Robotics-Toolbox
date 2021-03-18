function [aqp,bqp,zbf] = circzcbf(p,c,r,dc,dr,aqp,bqp)
% Circular constraints use zeroing control barrier function
if nargin<7
    aqp = [];
    bqp = [];
    if nargin<5
        dc = zeros(size(c));
        dr = zeros(size(r));
    end
end
p = p(:);
c = c(:);
dc = dc(:);
% extended K-class function
akcf = 1;
ekcf = @(h) akcf*h;
% bounded distance constraints
zbf = (p-c)'*(p-c)-r^2;
cbfa = 2*(p-c)';
cbfb = cbfa*dc+2*r*dr-ekcf(zbf);
aqp = [aqp;cbfa];
bqp = [bqp;cbfb];
end