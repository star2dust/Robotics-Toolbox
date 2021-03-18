function Ad_g_ab = Adg(g_ab)
% Calculate Ad for g_ab (double or SE(3))
[R_ab,p_ab] = tr2rt(g_ab);
Ad_g_ab = [R_ab,skew(p_ab)*R_ab;
    zeros(3),R_ab];
end