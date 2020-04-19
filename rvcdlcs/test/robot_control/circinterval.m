function [thmin,thmax] = circinterval(thr,thl)

while sum(thl<=thr)
    thl = thl+2*pi*(thl<=thr);
end
thmax = thl;
thmin = thr;
end