function count = innodepoly(randloc,nodepoly,inpolyerr)
polynum = length(nodepoly);
count = 0;
for i=1:polynum
    if nodepoly(i).A*randloc'-nodepoly(i).b<=inpolyerr
        count = count+1;
    end
end
end