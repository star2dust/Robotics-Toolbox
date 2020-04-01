close all
clear

import mpr.*
mu = @(q) -manipulability(q,ones(size([0;q(:)])));
m = 9;
q = fmincon(mu,zeros(m,1),[],[],[],[],zeros(m,1),pi/2*ones(m,1));
sum(q)