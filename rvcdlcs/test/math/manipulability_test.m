close all
clear

n = 3;
link = ones(1,n)*0.95;
negmu = @(q) -PlanarRevolute.getMu(link,q);
[q,f]= fmincon(negmu,zeros(n,1),[],[],[],[],zeros(n,1),pi/2*ones(n,1));
-f