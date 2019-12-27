%Example of conversion between DH parameters and POE parameters

%Please Refer to:

% Liao Wu, Ross Crawford, Jonathan Roberts. An analytic approach to
% converting POE parameters into D-H parameters for serial-link robots. IEEE
% Robotics and Automation Letters.

close all
clear
clc
%% prepare data
% DH parameters of PUMA robot are retrieved from Peter Corke. Robotics
% Vision and Control: Fundamental algorithms in MATLAB. 2011. 

H_base = eye(4)
H_tool = eye(4)
%DH_puma_std = [theta, d, alpha, a]
DH_puma_std =   [0      0       pi/2    0;
                0       0       0       0.4318;
                0       0.15    -pi/2   0.0203;
                0       0.4318  pi/2    0;
                0       0       -pi/2   0;
                0       0       0       0]
%DH_puma_mdf = [alpha, a, theta, d]
% DH_puma_mdf =   [0      0       0       0;
%                 pi/2    0       0       0;
%                 0       0.4318  0       0.15;
%                 -pi/2   0.0203  0       0.4318;
%                 pi/2    0       0       0;
%                 -pi/2   0       0       0]

%% conversion from DH to POE
POE_puma = DH2POE(DH_puma_std,H_base,H_tool,'RRRRRR','std')

%POE_puma = DH2POE(DH_puma_mdf,H_base,H_tool,'RRRRRR','mdf')

%% conversion from POE to DH

DH_puma_uni = POE2DH(POE_puma)

%% forward kinematics validation

q = ones(6,1);

fkDH_puma_std = fkDH(DH_puma_std,H_base,H_tool,q,'RRRRRR','std')

fkPOE_puma = fkPOE(POE_puma,q)

fkDH_puma_uni = fkDH(DH_puma_uni(2:7,:),DH(DH_puma_uni(1,:),'std'),DH([DH_puma_uni(8,1:2),0,0],'std'),q,'RRRRRR','std')