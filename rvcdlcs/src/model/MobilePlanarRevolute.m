% Mobile Planar Revolute Robot 3D Model class (rpy, stdDH)
% (last mod.: 05-01-2020, Author: Chu Wu)
% Requires rvc & rte https://github.com/star2dust/Robotics-Toolbox
% Properties:
% - name: str (mpr*)
% - type: str (elbowup* or elbowdown)
% - base: Cuboid
% - arm: SerialLink
% - mount: SE3 (on the upper surface of mobile base)
% - link: length of each link
% Methods:
% - MobilePlanarRevolute: construction (opt: name, type) (arg: edge, link, mount)
% - plot (opt: workspace, [no]frame, framecolor)
% - animate
% - fkine: forward kinematics
% - manipulability: manipulability and its derivative
% Methods (Static): (for mR manipulator)
% - mrtwist: calculate twists by link length and mounted base
% - mrfkine: forward kinematics
% - mrjacob: Jacobian
% - mrikine: inverse kinematics
classdef MobilePlanarRevolute < MobileRobot
    properties
        type
        link
    end
    
    methods
        function obj = MobilePlanarRevolute(varargin)
            % MPR.MobilePlanarRevolute  Create m-dof MobileRevolute robot object
            
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
        
        function h = plot(obj,varargin)
            % MPR.plot  Plot m-dof MobileRevolute robot object
            
            % opt statement
            opt.workspace = [];
            opt.frame = false;
            opt.framecolor = 'b';
            opt.framelength = sum(obj.base.edge)/length(obj.base.edge)/3;
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
            % q (3+m) => (6+m)
            q = obj.min2config(q);
            if opt.frame
                h = obj.plot@MobileRobot(q,'workspace',opt.workspace,'frame','framecolor',opt.framecolor,'framelength',opt.framelength, 'framethick', opt.framethick, 'framestyle', opt.framestyle);
            else
                h = obj.plot@MobileRobot(q,'workspace',opt.workspace);
            end
        end
        
        function animate(obj,q)
            % MPR.animate  Animate m-dof MobileRevolute robot object
            
            % q (3+m) => (6+m)
            q = obj.min2config(q);
            obj.animate@MobileRobot(q);
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
        
        
        function q = ikine(obj,tool,base,el)
            % MPR.ikine  Calculate inverse kinematics by tool, base (SE3) for mR manipulator
            
            % mobile base
            qb = SE3(base).toqrpy;
            % update pose
            obj.arm.base = SE3(base)*obj.mount;
            % manipulator
            if obj.arm.n>3
                qa = obj.arm.ikine(tool,'mask',[1,1,0,0,0,1]);
            else
                if nargin<4
                   el = 'elbowup';
                end
                T = obj.arm.base.inv*tool;
                qtool = T.toqrpy;
                qa = obj.mrikine(obj.link,qtool([1:2,6]),el);
            end
            % q (m+6) => (m+3)
            q = obj.config2min([qb,qa]);
        end
        
        
        function varargout = manipulability(obj,th)
            % MPR.manipulability   Calculate manipulability and its derivative 
            % for m-dof planar revolute manipulator. If there is no input argument, 
            % then calculate the maximum manipulability and the joint angles that
            % perserve the corresponding manipulability.
            
            m = obj.arm.n; l = obj.link;
            % maximun manipulability and joints
            if nargin<2              
                negmupower2 = @(th) -det(MobilePlanarRevolute.mrjacob(l,th)*MobilePlanarRevolute.mrjacob(l,th)');
                [thmax,mumax] = fmincon(negmupower2,zeros(1,m),[],[],[],[],zeros(1,m),ones(1,m)*pi/2);
                delta = [-ones(1,m-1);eye(m-1)];
                if obj.type=="elbowup"
                    thmax = -thmax(2:end)*delta';
                else
                    thmax = thmax(2:end)*delta';
                end
                varargout = {sqrt(-mumax),thmax};
            else
                % Jacobian
                J = MobilePlanarRevolute.mrjacob(l,th);
                dx = J(1,:)'; dy = J(2,:)';
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
                varargout = {mu,nablamu};
            end
        end
    end
    
    methods (Static)
        
        function xi_mat = mrtwist(link,mount)
            % MPR.mrtwist  Calculate POE twist (6 x m+1) by link for mR manipulator
            % (link: 1xm, mount: 1x3 translation from mobile base)
            
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
        
        
        function f = mrfkine(link,th)
            % MPR.mrfkine  Forward kinematics for m-dof planar revolute manipulator
            m = length(th);
            ls = zeros(m,1);
            lc = ls;
            % lsin and lcos
            f = [0;0];
            for i=1:m
                ls(i) = link(i)*sin(sum(th(1:i)));
                lc(i) = link(i)*cos(sum(th(1:i)));
                f = f+[lc(i);ls(i)];
            end
        end
        
        
        function th = mrikine(link,T,el)
            % MPR.mrikine  Inverse kinematics for 2 or 3-dof planar revolute manipulator
            if length(link)==3
                l1 = link(1); l2 = link(2); l3 = link(3);
                end_SE3 = SE2(T)*SE2([-l3,0]);
            elseif length(link)==2
                l1 = link(1); l2 = link(2);
                end_SE3 = SE2(T);
            else
                error('link length exceeds the degree of freedom (max 3)');
            end
            gT = end_SE3.q; 
            x = gT(1); y = gT(2); thT = gT(3);
            % angle => inverse kinematics of three-link planar robot
            if nargin<3
               el = 'elbowup'; 
            end
            switch el
                case 'elbowup' % up-elbow
                    ue = -1;
                case 'elbowdown' % down-elbow
                    ue = 1;
                otherwise
                    error('invalid elbow type');
            end
            c2 = (x^2+y^2-l1^2-l2^2)/(2*l1*l2);
            th2 = acos(c2)*ue;
            s2 = sqrt(1-c2^2);
            th1 = atan2(y,x)-atan2(l2*s2,l1+l2*c2)*ue;
            if length(link)==3
                th3 = thT-th1-th2;
                th = [th1,th2,th3];
            else
                th = [th1,th2];
            end
        end
        
        
        function J = mrjacob(link,th)
            % MPR.mrjacob  Jacobian for m-dof planar revolute manipulator
            m = length(th);
            ls = zeros(m,1);
            lc = ls; dx = ls; dy = ls;
            % lsin and lcos
            for i=1:m
                ls(i) = link(i)*sin(sum(th(1:i)));
                lc(i) = link(i)*cos(sum(th(1:i)));
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
    end
    
    methods (Access = protected)
        function q = min2config(obj,q)
            % q (3+m) => (6+m)
            if length(q)==obj.arm.n+3
                q = q(:)'; q = [q(1:2),obj.base.edge(3)/2,0,0,q(3:end)];
            end
        end
        
        function q = config2min(obj,q)
            % q (6+m) => (3+m)
            if length(q)==obj.arm.n+6
                q = q(:)'; q = [q(1:2),q(6:end)];
            end
        end
    end
end