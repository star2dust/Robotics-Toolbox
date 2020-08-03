% Mobile Planar Revolute Robot 3D Model class (SE2, rpy, stdDH)
% (last mod.: 01-04-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str (mpr*)
% - base: Platform
% - arm: SerialLink
% - mount: SE3 (on the upper surface of mobile base)
% - link: length of each link
% Methods:
% - MobilePlanarRevolute: construction (opt: name, type) (arg: edge, link, mh) 
%    (mh = [mount.t(1:2),height])
% - plot (opt: workspace, [no]frame, framecolor) (minConfig)
% - animate
% - fkine: forward kinematics
% - ikine: inverse kinematics
classdef MobilePlanarRevolute < MobileRobot
    properties
        link
    end
    
    methods
        function obj = MobilePlanarRevolute(varargin)
            % MPR.MobilePlanarRevolute  Create m-dof MobileRevolute robot object
            
            % opt statement
            opt.name = 'mpr';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);
            % check validity
            if length(arg)==3
                edge = arg{1}(:)';
                link = arg{2}(:)';
                % mh(1:2): mounted place on mobile base
                % mh(3): end-effector height above the upper surface
                mh = arg{3}(:)';
            else
                error('unknown argument')
            end
            % mounted place (translation only)
            if ~length(mh)==3||abs(mh(1))>edge(1)/2||abs(mh(2))>edge(2)/2
                error('invalid mounted place');
            end
            cub_surf= [mh(1),mh(2),edge(3)/2]; hb_d = mh(3); 
            % choose q according to type of joints
            import PlanarRevolute.*
            xi_mat = getTwist(link,hb_d,SE3);
            % poe => dh modified
            [dh, Ht, sigma] = poe2dh(xi_mat,hb_d); % qb only for theta or d
            Hb = transl(cub_surf); % Hb can only be on the surface of mobile base
            % limit
            m = length(link);
            Qlim = [-ones(m,1)*pi/2,ones(m,1)*pi/2];
            % construct MobileRobot
            obj = obj@MobileRobot(edge,[dh,sigma],Hb,Ht,'name',opt.name,'qlim',Qlim);
            obj.link = link;
        end
        
        function h = plot(obj,varargin)
            % MPR.plot  Plot m-dof MobileRevolute robot object
            
            % opt statement
            opt.workspace = [];
            opt.frame = false;
            opt.framecolor = 'b';
            opt.framelength = sum(obj.base.body.edge)/length(obj.base.body.edge)/3;
            opt.framethick = 1;
            opt.framestyle = '-';
            % opt parse: only stated fields are chosen to opt, otherwise to arg
            [opt,arg] = tb_optparse(opt, varargin);
            % argument parse
            if length(arg)==1
                % get pose
                q = arg{1}(:)';
            else
                error('unknown arguments');
            end
            if opt.frame
                h = obj.plot@MobileRobot(q,'workspace',opt.workspace,'frame','framecolor',opt.framecolor,'framelength',opt.framelength, 'framethick', opt.framethick, 'framestyle', opt.framestyle);
            else
                h = obj.plot@MobileRobot(q,'workspace',opt.workspace);
            end
        end
        
        function animate(obj,q,h)
            % MPR.animate  Animate m-dof MobileRevolute robot object
            
            if nargin < 3
                h = findobj('Tag', obj.name);
            end
            obj.animate@MobileRobot(q,h);
        end
            
        function [tool, base] = fkine(obj,q)
            % MPR.fkine  Calculate forword kinematics by q (3+m) for mR manipulator
            
            % q (3+m) => (6+m)
            q = obj.min2config(q);
            % get pose
            qa = q(7:end); qb = q(1:6);
            obj.arm.base = SE3.qrpy(qb)*obj.mount;
            % mobile base
            base = SE3.qrpy(qb);
            % manipulator
            tool = obj.arm.fkine(qa);
        end
        
        
        function q = ikine(obj,tool,base)
            % MPR.ikine  Calculate inverse kinematics by tool, base (SE3) for mR manipulator
            
            % mobile base
            qb = SE3(base).toqrpy;
            % update pose
            obj.arm.base = SE3(base)*obj.mount;
            % manipulator
            qa = obj.arm.ikine(tool,'mask',[1,1,0,0,0,1]);
            % q (m+6) => (m+3)
            q = obj.config2min([qb,qa]);
        end
    end
    
    methods (Access = protected)
        function q = min2config(obj,q)
            % q (3+m) => (6+m)
            if length(q)==obj.arm.n+3
                q = q(:)'; q = [q(1:2),obj.base.body.edge(3)/2+obj.base.body.edge(1)/6,0,0,q(3:end)];
            end
        end
        
        function q = config2min(obj,q)
            % q (6+m) => (3+m)
            if length(q)==obj.arm.n+6
                q = q(:)'; q = [q(1:2),q(7:end)];
            end
        end
    end
end