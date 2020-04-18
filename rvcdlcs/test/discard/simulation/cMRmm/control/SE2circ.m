function pout = SE2circ(pose,pin)
for i=1:size(pose,2)
    pout = h2e(SE2(pose(:,i)).T*e2h(pin));
end
end