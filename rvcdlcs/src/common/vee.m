function V = vee(Vh)
% transfer twist Vh to local representation V
[wh,v]=tr2rt(Vh);
V = [v;vex(wh);];
end