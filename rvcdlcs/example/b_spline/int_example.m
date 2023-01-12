 
clc;clear;
 
x=[0,0.4,1,2,pi];
y=sin(x);
 
%建立三次样条函数并积分
sp1=csapi(x,y);
a=fnint(sp1,1);
xx1=fnval(a,[0,pi]);
integral1=xx1(2)-xx1(1)
 
%建立B样条函数并积分
sp2=spapi(5,x,y);
b=fnint(sp2,1);
xx2=fnval(b,[0,pi]);
integral2=xx2(2)-xx2(1)
 
%绘制曲线
fplot(@(t) -cos(t)+2,[0,pi]); %不定积分可以上下平移
hold on,
fnplt(a,'--');
fnplt(b,':');