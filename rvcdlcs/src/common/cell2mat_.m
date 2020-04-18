function m = cell2mat_(c,x,y)

if isa(c,'cell')
    m = [];
    for i=1:length(c)
       m = [m;c{i}(x,y)];
    end
else
    error('unknown argument');
end
end