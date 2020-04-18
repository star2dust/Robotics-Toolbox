disp('Robotics Toolbox Extensions: (c) star2dust 2019-2020')

% quick start for the robotics toolbox extensions
srcpath = fileparts(mfilename('fullpath'));
addpath(srcpath); % add src folder to the matlab path
addpath([srcpath '/common']); % add common
addpath([srcpath '/robotics']); % add model
addpath([srcpath '/planning']); % add planning

% import third party toolboxes
thirdpartypath = [srcpath '/thirdparty'];
addpath([thirdpartypath '/cvxhull/lib']); % add cvxhull
addpath([thirdpartypath '/dh2poe/lib']); % add dh2poe
addpath([thirdpartypath '/graphsearch/lib']); % add graphsearch
addpath([thirdpartypath '/stlreader/lib']); % add stlreader
addpath([thirdpartypath '/iris-distro/src/matlab']); % add iris
addpath([thirdpartypath '/hebinput/hebi']); % add hebi input
%addpath([thirdpartypath '/circles']); % add circles


