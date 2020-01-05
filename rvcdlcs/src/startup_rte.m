disp('Robotics Toolbox Extensions: (c) star2dust 2019')

% quick start for the robotics toolbox extensions
srcpath = fileparts(mfilename('fullpath'));
addpath(srcpath); % add src folder to the matlab path
addpath([srcpath '/common']); % add common
addpath([srcpath '/model']); % add model
addpath([srcpath '/planning']); % add planning

% import third party toolboxes
thirdpartypath = [srcpath '/thirdparty'];
addpath([thirdpartypath '/cvxhull']); % add cvxhull
addpath([thirdpartypath '/graphsearch']); % add graphsearch
addpath([thirdpartypath '/stlreader']); % add stlreader
addpath([thirdpartypath '/iris-distro/src/matlab']); % add iris
addpath([thirdpartypath '/hebinput/hebi']); % add hebi input



