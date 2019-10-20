% scara robot inverse kinematics calcaltion
% reference: https://nl.mathworks.com/matlabcentral/fileexchange/69892-inverse-kinematics-of-a-2-link-robot-arm
clear
close all

%% parameters
% link length
syms l1 l2 l3 l4 real
% joint angle
syms th1 th2 th3 th4 real
% tool configuration
syms xt yt zt pht real
%% forward kinematics
xt_eq = xt==l1*cos(th1)+l2*cos(th1+th2)+l4*cos(th1+th2+th4);
yt_eq = yt==l1*sin(th1)+l2*sin(th1+th2)+l4*sin(th1+th2+th4);
zt_eq = zt == l3-th3;
pht_eq = pht == th1+th2+th4;
%% solve for inverse kinematics
ths = solve([xt_eq,yt_eq,zt_eq,pht_eq],[th1,th2,th3,th4]);
ths_mat = [ths.th1,ths.th2,ths.th3,ths.th4];
matlabFunction(ths_mat, 'File', 'scaraikine','Vars',[l1 l2 l3 xt yt zt pht],'Outputs',{'ths_mat'});
pt = [l1*cos(th1)+l2*cos(th1+th2),l1*sin(th1)+l2*sin(th1+th2),th4];
Rt = rotz(th3);
tool_T = rt2tr(Rt,pt);
matlabFunction(tool_T, 'File', 'scarafkine','Vars',[l1 l2 l3 th1 th2 th3 th4],'Outputs',{'tool_T'});
