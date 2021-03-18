function V = vee(Vh)
% Transfer twist Vh to local representation V
[wh,v]=tr2rt(Vh);
V = [v;vex(wh);];
end