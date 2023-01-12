%创建一个具有四个区间的单个分段多项式，这些区间在两个二次多项式之间交替。
clc;clear;
 
%显示一个二次多项式在[-8,-4]区间上的结果
subplot(2,2,1)
cc=[-1/4 1 0];
pp1=mkpp([-8 -4],cc);
xx1=-8:0.1:-4;
plot(xx1,ppval(pp1,xx1),'k-')
 
%在[-4,0]区间上的求反
subplot(2,2,2)
pp2=mkpp([-4 0],-cc);
xx2=-4:0.1:0;
plot(xx2,ppval(pp2,xx2),'k-')
 
%将二次多项式扩展到四个区间形成的分段多项式
%显示一阶导数，该导数利用unmkpp分解分段多项式构造而成
subplot(2,1,2)
pp=mkpp([-8 -4 0 4 8],[cc;-cc;cc;-cc]);
xx=-8:0.1:8;
plot(xx,ppval(pp,xx),'k-')
[breaks,coefs,k,d]=unmkpp(pp);
dpp=mkpp(breaks,repmat(k-1:-1:1,d*1,1).*coefs(:,1:k-1),d);