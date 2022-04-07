function bound = anglelim(loc,envir)

if loc(1)>=envir.lb(1)&&loc(1)<=-2&&loc(2)>=envir.lb(2)&&loc(2)<=0
    bound = [0,pi/2];
elseif loc(1)>=-2&&loc(1)<=10&&loc(2)>=envir.lb(2)&&loc(2)<=0
    bound = [-pi/2,0];
elseif loc(1)>=10&&loc(1)<=envir.ub(1)&&loc(2)>=envir.lb(2)&&loc(2)<=envir.ub(2)
bound = [0,2*pi/3];
elseif loc(1)>=envir.lb(1)&&loc(1)<=10&&loc(2)>=0&&loc(2)<=envir.ub(2)
    bound = [2*pi/3,4*pi/3];
end
end