function y = proj_circle(x,x0,r)
if norm(x-x0)>r
    y = r*(x-x0)/norm(x-x0);
else
    y = x;
end
end