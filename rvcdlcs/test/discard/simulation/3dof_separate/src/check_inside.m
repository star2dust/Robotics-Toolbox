function flag = check_inside(pt_check,pt_intsec)
x = pt_check(1);
y = pt_check(2);
c1 = -x-y-pi/2;
c2 = y;
k1 = -1/(1+sin(pt_intsec(2))/sin(sum(pt_intsec)));
b1 = pt_intsec(2)-k1*pt_intsec(1);
k2 = -1/(1+cos(pt_intsec(2))/cos(sum(pt_intsec)));
b2 = pt_intsec(2)-k2*pt_intsec(1);
c3 = y-k1*x-b1;
c4 = k2*x+b2-y;
c = max(c1,max(c2,max(c3,c4)));
if c<0.0001
    flag = true;
else
    flag = false;
end
end