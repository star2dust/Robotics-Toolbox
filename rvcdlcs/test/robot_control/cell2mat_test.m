close all
clear

A = [1,2,3;
    4,5,6];
for i=1:4
   c{i} = A*(i-1);
end

c1 = cell2mat_(c,[1 2],1)

c2 = cell2mat_(c,1,[1 2])

c3 = cell2mat_(c,1,[1 2],1)

% c1 =
% 
%      0     1     2     3
%      0     4     8    12
% 
% 
% c2 =
% 
%      0     0     1     2     2     4     3     6
% 
% 
% c3 =
% 
%      0     0
%      1     2
%      2     4
%      3     6