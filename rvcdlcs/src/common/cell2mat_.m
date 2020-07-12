function m = cell2mat_(ceq,x,y,p)

if nargin<4
   p = 2; 
end

if p==2
    cm = [ceq{:}];
else
    cm = [];
    for i=1:length(ceq)
       cm = [cm;ceq{i}]; 
    end
end

if isa(ceq,'cell')
    n = size(ceq{1},p);
    i = 1:n:n*length(ceq);
    if p==2
        m = cm(x,kron(i,ones(size(y)))+kron(ones(size(i)),y-1));
    else
        m = cm(kron(i,ones(size(x)))+kron(ones(size(i)),x-1),y);
    end
else
    error('unknown argument');
end
end