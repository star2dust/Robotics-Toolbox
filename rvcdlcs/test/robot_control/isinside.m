function f = isinside(q,A,b,qlim)
if isa(A,'cell')
    for i=1:size(q,1)
        f(i,:) = ~(sum(A{i}*q(i,:)'-b{i}>0)||sum(q(i,:)-qlim{i}(1,:)<0)||sum(q(i,:)-qlim{i}(2,:)>0));
    end
else
    for i=1:size(q,1)
        f(i,:) = ~(sum(A*q(i,:)'-b>0)||sum(q(i,:)-qlim(1,:)<0)||sum(q(i,:)-qlim(2,:)>0));
    end
end    
end