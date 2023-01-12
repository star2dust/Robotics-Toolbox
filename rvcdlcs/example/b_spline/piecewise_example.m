%创建任意一个分段多项式，使得它在区间[0,4]内具有三次多项式，在区间[4,10]内具有二次多项式，在区间[10,15]内具有四次多项式。
clc;clear;
 
breaks=[0 4 10 15];
coefs=[0 1 -1 1 1;0 0 1 -2 53;-1 6 1 4 77]; %一共有三段
pp=mkpp(breaks,coefs)
 
%画图
xq=0:0.01:15;
plot(xq,ppval(pp,xq))
line([4 4],ylim,'LineStyle','--','Color','k')
line([10 10],ylim,'LineStyle','--','Color','k')