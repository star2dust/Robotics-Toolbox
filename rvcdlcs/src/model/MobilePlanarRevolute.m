% Mobile Planar Revolute Robot 3D Model class (rpy, stdDH)
% (last mod.: 05-01-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str (mpr*)
% - type: str (elbowup* or elbowdown)
% - base: Cuboid
% - arm: SerialLink
% - mount: SE3 (from base to first link)
% - link: length of each link
% Methods:
% - MobilePlanarRevolute: construction (opt: name, type) (arg: edge, link, mount)
% - plot (opt: workspace, [no]frame, framecolor)
% - animate
% - fkine: forward kinematics
% Methods (Static): (for mR manipulator)
% - mrtwist: calculate twists by link length and mounted base
% - mrfkine: translation forward kinematics and Jacobian
% - mrmanipulability: manipulability and its derivative
classdef MobilePlanarRevolute < MobileRobot
    properties
        type
        link
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
            cub_surf= [mh(1),mh(2),edge(3)/2];
            hb_d = mh(3); hb_T = transl([0,0,hb_d]);
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
            [dh, Ht, sigma] = poe2dh(xi_mat,hb_d); % qb only for theta or d 
            Hb = transl(cub_surf); % Hb can only be on the surface of mobile base
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
            obj.link = link;
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
            q = obj.min2config(q);
            if opt.frame
                h = obj.plot@MobileRobot(q,'workspace',opt.workspace,'frame','framecolor',opt.framecolor);
            else
                h = obj.plot@MobileRobot(q,'workspace',opt.workspace);
            end           
        end 
        
        function animate(obj,q)
            % q (3+m) => (6+m)
            q = obj.min2config(q);
            obj.animate@MobileRobot(q);
        end
        
        % calculate forword kinematics by q (3+m) for mR manipulator
        function [tool, base] = fkine(obj,q)
            % q (3+m) => (6+m)
            q = obj.min2config(q);
            % update pose
            qa = q(7:end); qb = q(1:6);
            obj.arm.base = SE3.qrpy(qb)*obj.mount;
            % mobile base
            base = SE3.qrpy(qb);
            % manipulator
            tool = obj.arm.fkine(qa);
        end
    end
    
    methods (Static)
        % calculate POE twist (6 x m+1) by link for mR manipulator 
        % (link: 1xm, mount: 1x3 translation from mobile base)
        function xi_mat = mrtwist(link,mount) 
            % mounted place (translation only for mR) 
            % indeed hb_T can be chosen as any transformation
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
        end
        
        % forward kinematics for m-dof planar revolute manipulator
        function [f,J] = mrfkine(th,link)
            m = length(th);
            ls = zeros(m,1);
            lc = ls; dx = ls; dy = ls;
            % lsin and lcos
            f = [0;0];
            for i=1:m
                ls(i) = link(i)*sin(sum(th(1:i)));
                lc(i) = link(i)*cos(sum(th(1:i)));
                f = f+[lc(i);ls(i)];
            end
            % dxdth and dydth
            for i=1:m
                dx(i) = 0;
                dy(i) = 0;
                for j=i:m
                    dx(i) = dx(i)-ls(j);
                    dy(i) = dy(i)+lc(j);
                end
            end
            % Jacobian
            J = [dx,dy]';
        end
        
        % calculate manipulability and its derivative for m-dof planar revolute manipulator
        function [mu,nablamu] = mrmanipulability(th,link)
            % Jacobian
            [~,J] = mrfkine(th,link);
            % mu^2
            mu = sqrt(det(J*J'));
            % second method to calcuate row vec of Hessian
            Hdy = kron(ones(1,m),dy);
            Hdx = kron(ones(1,m),dx);
            for i=1:m-1
                for j=i:m
                    Hdy(i,j) = dy(j);
                    Hdx(i,j) = dx(j);
                end
            end
            rvecH = [-Hdy,Hdx];
            % nabla mu
            JJTinv = (J*J')^-1;
            nablamu = mu*rvecH*kron(eye(2),J')*JJTinv(:);
        end
    end
    
    methods (Access = protected) 
        function q = min2config(obj,q)
            % q (3+m) => (6+m)
            if length(q)==obj.arm.n+3
                q = q(:)'; q = [q(1:2),obj.base.edges(3)/2,0,0,q(3:end)];
            end
        end
    end
end