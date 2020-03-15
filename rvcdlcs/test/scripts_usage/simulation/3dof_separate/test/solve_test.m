close all
clear


syms x y k1 b1 k2 b2 px py real
% constraints
c1 = -x-y-pi/2;
c2 = y;
c3 = y-k1*x-b1;
c4 = k2*x+b2-y;
% rs solve
rs1 = solve(c3,c4,x,y);
rs2 = solve(c1,c4,x,y);
% constraints vector
v0 = [px;py]-[x;y];
v1 = [1;-1];
v2 = [-1;0];
v3 = [1;k1];
v4 = [1;k2];
% re solve
re1 = solve(v0'*v1,c1,x,y);
re2 = solve(v0'*v3,c3,x,y);
re3 = solve(v0'*v4,c4,x,y);