function g_ab_inv = invg(g_ab)
% Calculate inverse of g_ab (double or SE(3))
[R_ab,p_ab] = tr2rt(g_ab);
g_ab_inv = [R_ab',-R_ab'*p_ab;
    zeros(1,3),1];
end