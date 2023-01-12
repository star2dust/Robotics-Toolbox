clc;clear;
 
syms x;
f=(x^2-3*x+5)*exp(-5*x)*sin(x);
fplot(diff(f),[0,1]) %理论结果
hold on,
x=0:.12:1;
y=(x.^2-3*x+5).*exp(-5*x).*sin(x);
sp1=csapi(x,y); %建立三次样条函数
dsp1=fnder(sp1,1);
fnplt(dsp1,'--'); %绘制样条图
hold on,
 
sp2=spapi(5,x,y); %5阶次B样条插值
dsp2=fnder(sp2,1);
fnplt(dsp2,':');
axis([0,1,-0.8,5])