close all
clear

import mpr.*

m = 3; l = ones(1,m);
mu = @(q) -manipulability(q,l);
q = fmincon(mu,zeros(1,m-1),[],[],[],[],zeros(1,m-1),pi/2*ones(1,m-1));
q/pi*180