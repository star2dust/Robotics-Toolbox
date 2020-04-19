close all
clear

load('Stest.mat','S');
for i=1:length(S)
    figure
    plot(S{i}(:,1),S{i}(:,2),'b*');
    k = convhull_(S{i});
end
