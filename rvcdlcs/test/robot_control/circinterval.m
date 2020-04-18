function [thmin,thmax] = circinterval(thr,thl)

while thl<=thr
    thl = thl+2*pi;
end
thmax = thl;
thmin = thr;
end