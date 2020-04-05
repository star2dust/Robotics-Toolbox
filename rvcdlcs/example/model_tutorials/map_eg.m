close all
clear

import Map.*

% m = randMap([5,5],0.5);
m = [1   0   0   1   0;
     1   0   1   1   0;
     0   0   0   0   1;
     1   0   0   0   0;
     0   1   1   1   1];

a = Map(m,'name','map1');
a.plot;