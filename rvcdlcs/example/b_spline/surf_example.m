clc;clear;
 
%拟合曲面
x0=-3:.3:3;
y0=-2:.2:2;
[x,y]=ndgrid(x0,y0);
z=(x.^2-2*x).*exp(-x.^2-y.^2-x.*y);
sp=spapi({5,5},{x0,y0},z); %B样条
dspxy=fnder(sp,[1,1]);
fnplt(dspxy) %生成样条图
 
%理论方法
syms X Y;
Z=(X^2-2*X)*exp(-X^2-Y^2-X*Y);
figure;
fsurf(diff(diff(Z,X),Y),[-3 3 -2 2])
%对符号变量表达式做三维表面图