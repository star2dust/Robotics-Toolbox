disp('Robotics Toolbox Extensions: (c) star2dust 2019')

% quick start for the robotics toolbox extensions
rtepath = fileparts(mfilename('fullpath'));
srcpath = fullfile(rtepath, 'src/');
addpath(srcpath); % add src folder to the matlab path
addpath([srcpath 'common']); % add common

% import third party toolboxes
thirdpartypath = [srcpath 'thirdparty/'];
addpath([thirdpartypath 'cvxhull']); % add cvxhull
addpath([thirdpartypath 'graphsearch']); % add graphsearch
addpath([thirdpartypath 'stlreader']); % add stlreader
addpath([thirdpartypath 'iris-distro/src/matlab']); % add iris


