disp('Robotics Toolbox Extension: (c) Chu Wu 2019-2020')

if ~isdeployed
    % quick start for the toolbox
    srcpath = fileparts(mfilename('fullpath'));
    addpath(srcpath); % add src folder to the matlab path
    addpath([srcpath '/common']); % add common
    addpath([srcpath '/robotics']); % add robotics
    addpath(genpath([srcpath '/planning'])); % add planning
    addpath([srcpath '/multiagent']); % add robotics
    
    % import third party toolboxes
    thirdpartypath = [srcpath '/thirdparty'];
    % add cvxhull
    addpath([thirdpartypath '/cvxhull/circles']);
    addpath([thirdpartypath '/cvxhull/inhull']);
    addpath([thirdpartypath '/cvxhull/inpoly']);
    addpath([thirdpartypath '/cvxhull/intersections']);
    addpath([thirdpartypath '/cvxhull/polytopes']);
    % add graphsearch
    addpath([thirdpartypath '/graphsearch/lib']);
    % add hebi input
    addpath([thirdpartypath '/hebinput/hebi']);
    % add dh2poe
    addpath([thirdpartypath '/dh2poe/lib']);
    % add stlreader
    addpath([thirdpartypath '/stlreader/lib']);
    % add iris
    addpath([thirdpartypath '/iris-distro/src/matlab']);
    % add visual
    addpath(genpath([thirdpartypath '/coppelia/visual']));
    % add hungarian
    addpath(genpath([thirdpartypath '/hungarian/lib']));
    % add jsonlab
    addpath([thirdpartypath '/jsonlab']);
end


