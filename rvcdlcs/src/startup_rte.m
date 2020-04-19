disp('Robotics Toolbox Extensions: (c) star2dust 2019-2020')

% quick start for the robotics toolbox extensions
srcpath = fileparts(mfilename('fullpath'));
addpath(srcpath); % add src folder to the matlab path
addpath([srcpath '/common']); % add common
addpath([srcpath '/robotics']); % add model
addpath([srcpath '/planning']); % add planning

% import third party toolboxes
thirdpartypath = [srcpath '/thirdparty'];

% add cvxhull  
addpath([thirdpartypath '/cvxhull/circles']);
addpath([thirdpartypath '/cvxhull/inhull']); 
addpath([thirdpartypath '/cvxhull/inpoly']); 
addpath([thirdpartypath '/cvxhull/intersections']); 
addpath([thirdpartypath '/cvxhull/polytopes']); 

% add dh2poe 
addpath([thirdpartypath '/dh2poe/lib']); 

% add graphsearch
addpath([thirdpartypath '/graphsearch/lib']); 

% add hebi input
addpath([thirdpartypath '/hebinput/hebi']);

% add stlreader
addpath([thirdpartypath '/stlreader/lib']); 

% add iris
addpath([thirdpartypath '/iris-distro/src/matlab']); 


