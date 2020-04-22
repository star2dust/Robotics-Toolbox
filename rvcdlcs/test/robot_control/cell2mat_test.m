close all
clear

A = [1,2,3;4,5,6];
for i=1:4
   c{i} = A*(i-1);
end

c1 = cell2mat_(c,[1 2],1)

c2 = cell2mat_(c,1,[1 2])

c3 = cell2mat_(c,1,[1 2],1)