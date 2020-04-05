function vs = skew2(v)
% skew for 2-vector
if length(v)==2
    S = skew([v(:);0]);
    vs = S(3,1:2);
else
    vs = skew(v);
end
end