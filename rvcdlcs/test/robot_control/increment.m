function name = increment(name,type)
ctr = 0;
while exist([name type],'file')
    ctr = ctr+1;
    name = [name num2str(ctr)];
end
end