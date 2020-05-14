function f = isinside(q,A,b,qlim)
if isa(A,'cell')
    for i=1:size(q,1)
        linieq = sum(~(A{i}*q(i,:)'-b{i}<10^-4));
        limieq = sum(~(q(i,:)-qlim{i}(1,:)>-10^-4&q(i,:)-qlim{i}(2,:)<10^-4));
        f(i,:) = ~(linieq||limieq);
    end
else
    for i=1:size(q,1)
        linieq = sum(~(A*q(i,:)'-b<10^-4));
        limieq = sum(~(q(i,:)-qlim(1,:)>-10^-4&q(i,:)-qlim(2,:)<10^-4));
        f(i,:) = ~(linieq||limieq);
    end
end    
end