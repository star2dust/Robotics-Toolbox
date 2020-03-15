close all
clear

count = 0;
for t=1:0.05:2
    count = count+1;
    q3(count,:) = t*[ones(1,3),0.5*ones(1,3)];
    q2(count,:) = t*[ones(1,2),0.5];
end

% a = SE3.qeul(q3);
% a.toqeul

% a = SE3.qrpy(q);
% a.toqrpy


b = SE2(q2);
b.q