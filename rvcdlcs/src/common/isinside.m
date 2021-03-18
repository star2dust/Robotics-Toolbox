function f = isinside(q,A,b,lim,e)
% Check if q is inside a constraint set composed by A,b,qlim
if nargin<5
   e = 10^-4; 
end
if isa(A,'cell')
    % test each q for each A,b
    for i=1:size(q,1)
        linieq = sum(~(A{i}*q(i,:)'-b{i}<e));
        limieq = sum(~(q(i,:)-lim{i}(1,:)>-e&q(i,:)-lim{i}(2,:)<e));
        f(i,:) = ~(linieq||limieq);
    end
else
    % test each q for one A,b
    for i=1:size(q,1)
        linieq = sum(~(A*q(i,:)'-b<e));
        limieq = sum(~(q(i,:)-lim(1,:)>-e&q(i,:)-lim(2,:)<e));
        f(i,:) = ~(linieq||limieq);
    end
end    
end