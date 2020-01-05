% Create cooperative manipulation scenario
% (last mod.: 03-01-2020, author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% input [nrob,nmap,nload,opt]
% - opt: options ()
% output [rob, obs, load]
% - item: item structure (class)
% - init: possible initial states
function [rob, obs, load] = scenario(varargin)
% template settings
rob_edge_t = [0.8,0.6,0.5]; rob_dof_t = 2; 
rob_link_t = ones(1,rob_dof_t)*0.8; rob_mount_t = [0,0,rob_edge_t(3)/2+1];
rob_rad_t = sqrt(rob_edge_t(1)^2+rob_edge_t(2)^2)/2;
load_edge_t = [ones(1,2)*sqrt(2),0.2];
load_rad_t = sqrt(rob_edge_t(1)^2+rob_edge_t(2)^2)/2;
obs_edge_t = [ones(1,2)*(load_rad_t+sum(rob_link_t)*2/3+rob_rad_t)*2,0.5];
% save in params (rp, ld)
rp.arg = {rob_edge_t,rob_link_t,rob_mount_t}; % robot
lp.arg = {load_edge_t}; % load
% predefined options
opt.robfcn = @MobilePlannarRevolute;
opt.obsfcn = @Cuboid;
opt.loadfcn = @Cuboid;
opt.grid = ceil(obs_edge_t(1));
opt.pct = 0.35;
opt.dof = rob_dof_t;
% opt parse: only stated fields are chosen to opt, otherwise to arg
[opt,arg] = tb_optparse(opt, varargin);
% check validity
if length(arg)==3
    nrob = arg{1};
    nmap = arg{2};
    nload = arg{3};
elseif length(arg)==2
    nrob = arg{1};
    nmap = arg{2};
    nload = 1;
else
    error('unknown argument') 
end
if ~(opt.grid>=obs_edge_t(1)&&opt.dof>=rob_dof_t)
    error('invalid option') 
end
% generate obstacles 
obs_edge = [ones(1,2)*opt.grid,obs_edge_t(3)];
op.arg = {obs_edge}; % obstacle
[oq, map] = environment(op.arg{:}, nmap, opt.pct);
% update params
rp.num = nrob; lp.num = nload; op.num = size(oq,1);
% generate items
for i=1:nrob
    rob.item(i) = MobilePlanarRevolute(rp(i).arg{:},'name',['rob' num2str(i)],'type','elbowup');
end
for i=1:nmap
    obs.item(i) = Cuboid(op(i).arg{:},'name',['obs' num2str(i)]);
end
load.item = Cuboid(lp.arg{:},'name','load');
% generate initials
obs.init = oq;
load.init = [ones(1,2)*opt.grid/2,rob_mount_t(3)+rob_edge_t(3)/2,zeros(1,3)];

[rob.init,obs.init,load.init] = param2init(rp,op,lp);
end

function [qobs, map] = environment(edge,nmap,pct)
map = rand(nmap)>pct;
lx = edge(1); ly = edge(2); lz = edge(3);
qobs = [];
for i=1:nmap
    for j=1:nmap
        if map(i,j)==1
            xobs = (i-1)*lx+lx/2;
            yobs = nmap*ly-((j-1)*ly+ly/2);
            qobs = [qobs;xobs,yobs,lz/2,zeros(1,3)];
        end
    end
end
end

function g_ce = distrib(nrob)

end

% function [qrob] = ikine(qload,g_ce,qbase)
% 
% end