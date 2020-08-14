close all
clear

mdl_puma560
plotopt = {'nowrist'};
q = 1:6;
p560.plot3d(q,plotopt{:});