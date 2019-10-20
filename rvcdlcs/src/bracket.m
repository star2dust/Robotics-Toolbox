function c = bracket(a,b)
% calculate Lie bracket of twists a and b
ch = hatwedge(a)*hatwedge(b)-hatwedge(b)*hatwedge(a);
c = hatvee(ch);
end