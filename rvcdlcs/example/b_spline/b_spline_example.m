clc;clear;
 
%%y函数部分
x0=[0,0.4,1,2,pi];
y0=sin(x0);
fplot(@(t) sin(t),[0,pi]);
hold on

%原目标点
plot(x0,y0,'ro');
 
%三次分段多项式样条插值
sp1=csapi(x0,y0);
fnplt(sp1,'--');
 
%5次B样条插值
sp2=spapi(5,x0,y0); %k为用户选定的B样条阶次，一般以4和5居多
fnplt(sp2,':')
 
%%f(x)函数部分
x=0:.12:1;
y=(x.^2-3*x+5).*exp(-5*x).*sin(x);
figure;
fplot(@(x) (x.^2-3*x+5).*exp(-5*x).*sin(x),[0,1]),
hold on
plot(x,y,'ro');
sp1=csapi(x,y);
fnplt(sp1,'--');
sp2=spapi(5,x,y);
fnplt(sp2,':')