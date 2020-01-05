% Mobile Planar Revolute Robot 3D Model class (rpy, stdDH)
% (last mod.: 05-01-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str
% - type: str (elbowup* or elbowdown)
% - base: Cuboid
% - arm: SerialLink
% - mount: SE3 (from base to first link)
% - height: initial end-effector height above the ground
% Methods:
% - MobilePlanarRevolute: construction (opt: name, type) (arg: edge, link, mount)
% - plot (opt: workspace, [no]frame, framecolor)
% - animate
% - twist: calculate twists by link length and mounted base
classdef MobilePlanarRevolute < MobileRobot
    properties
        type
        height
    end
    
    methods
        function obj = MobilePlanarRevolute(varargin)
            % MobileRevolute creates m-dof MobileRevolute robot object
            % opt statement
            opt.type = 'elbowup';
            opt.name = 'mpr';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);
            % check validity
            if length(arg)==3
                edge = arg{1}(:)';
                link = arg{2}(:)';
                % mount(1:2): base coordinate on platform
                % mount(3): initial end-effector height
                mount = arg{3}(:)';
            else
               error('unknown argument') 
            end
            % mounted place (translation only)
            if ~length(mount)==3||abs(mount(1))>edge(1)/2||abs(mount(2))>edge(2)/2
               error('invalid mounted place'); 
            end
            cub_surf= [mount(1),mount(2),edge(3)/2];
            hb_d = mount-cub_surf; hb_T = transl(hb_d);
            % choose q according to type of joints
            l0 = [0;link(:)];
            for i=1:length(l0)
                xi_q(:,i) = [sum(l0(1:i)),0,0]';
%                 xi_q(:,i) = h2e(hb_T*e2h(xi_q(:,i)));
            end
            % rotation axis
            w = [0,0,1]';
            % joint twists
            for i=1:length(l0)-1
                xi_mat(:,i) = [-skew(w)*xi_q(:,i);w];
            end
            % tool twist
            g_st0 = hb_T*transl(xi_q(:,end));
            xi_mat(:,i+1) = vee(logm(g_st0));
            % poe => dh modified
            [dh, Ht, sigma] = poe2dh(xi_mat,hb_d(3)); % qb only for theta or d 
            Hb = transl(cub_surf);
            % type 
            m = length(link);
            switch opt.type
                case 'elbowdown'
                    Qlim = [zeros(m,1),ones(m,1)*pi/2];
                case 'elbowup'
                    Qlim = [-ones(m,1)*pi/2,zeros(m,1)];
                otherwise
                    error('Invalid type');
            end
            % construct MobileRobot
            obj = obj@MobileRobot(edge,[dh,sigma],Hb,Ht,'name',opt.name,'qlim',Qlim);
            obj.type = opt.type;
            obj.height = mount(3);
        end
        
        function h = plot(varargin)  
            % opt statement
            opt.workspace = [];
            opt.frame = false;
            opt.framecolor = 'b';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin); 
            % argument parse
            if length(arg)==2
                % get object
                obj = arg{1};
                % get pose
                q = arg{2}(:)';
            else
                error('unknown arguments');
            end
            % q (3+m) => (6+m)
            if length(q)==obj.arm.n+3
                q = q(:)'; q = [q(1:2),obj.base.edges(3)/2,0,0,q(3:end)];
            end
            if opt.frame
                h = obj.plot@MobileRobot(q,'workspace',opt.workspace,'frame','framecolor',opt.framecolor);
            else
                h = obj.plot@MobileRobot(q,'workspace',opt.workspace);
            end           
        end 
        
        function animate(obj,q)
            if length(q)==obj.arm.n+3
                q = q(:)'; q = [q(1:2),obj.base.edges(3)/2,0,0,q(3:end)];
            end
            obj.animate@MobileRobot(q);
        end
    end
    
    methods (Static)
        % calculate POE twist (6 x m+1) by l for mR manipulator
        function xi_mat = twist(link,base)
            % calculate twist (link: 1xm, base: 4x4 from origin)
            gs0 = base;
            l0 = [0;link(:)];
            for i=1:length(l0)
                q(:,i) = [sum(l0(1:i)),0,0]';
                q(:,i) = h2e(gs0*e2h(q(:,i)));
            end
            % rotation axis
            w = [0,0,1]';
            % joint twists
            for i=1:length(l0)-1
                xi_mat(:,i) = [-skew(w)*q(:,i);w];
            end
            % tool twist
            g_st0 = transl(q(:,end));
            xi_mat(:,i+1) = vee(logm(g_st0));
        end
        
        function q = ikine(tool, base)
            
        end
        
        % calculate POE twist (6 x m+1) by l for mR manipulator
        function [tool, base] = fkine(q)
            
        end
    end
    
    methods (Access = protected) 
        function h = createRobot(obj, q, opt)
            % create an axis
            ish = ishold();
            if ~ishold
                % if hold is off, set the axis dimensions
                if ~isempty(opt.workspace)
                    axis(opt.workspace);
                end
                hold on
            end
            % update pose
            qa = q(7:end); qb = q(1:6);
            obj.arm.base = SE3.qrpy(qb)*obj.mount;
            
            group = hggroup('Tag', obj.name);
            h.group = group;
            h.arm = obj.arm.plot(qa);
            if opt.frame
                h.base = obj.base.plot(qb,'frame','framecolor', opt.framecolor);
                h.tool = SE3(obj.arm.fkine(qa)).plot('color', opt.framecolor);
                set(h.tool,'parent',group);
                set(h.tool,'Tag', [obj.name '-tool']);
            else
                h.base = obj.base.plot(qb);
            end
            set(h.arm.group,'parent',group);
            set(h.base.group,'parent',group);
                
            % restore hold setting
            if ~ish
                hold off
            end
        end
    end
end