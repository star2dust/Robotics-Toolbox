% mR manipulator 2D Model class
% (last mod.: 22-12-2019, Author: Chu Wu)
% Requires Robotics Toolbox of Peter Corke http://petercorke.com/wordpress/toolboxes/robotics-toolbox
% Properties:
% - states: pose (6x1), frame (SE3)
% - dynamics parameters: mass, inertia, inerMat
% - shapes: verts, faces, edges
% Methods:
% - updateVerts
classdef MR2 < handle
    properties (SetAccess = protected) % all display variables are row vectors
        % translation & rotation states in world frame (m dim) 
        posture 
        % frames (SE2) 
        baseFrame
        toolFrame 
        % params
        mass % center of body frame (1 dim)
        inertia 
        inerMat % diag(m,m,Iz)
        % a list of verts and edges
        bverts % (4x2) (body frame)
        verts % (4x2) (inertia frame)
        faces % (1x4)
        edges % (1x2) depth(x) width(y)
    end
    
    properties (Constant, Access = private)
        templateVerts = [0,0;0,1;1,1;1,0];
        templateFaces = [1,2,3,4];
        % ^ y axis
        % | 6 % % 7 -> top
        % | % 2 3 % -> bottom
        % | % 1 4 % -> bottom
        % | 5 % % 8 -> top
        % -------> x axis
    end
    
    methods
        function obj = Cuboid2(inputPose,inputEdges)
            % basic configuration
            if isvector(inputPose)&&length(inputPose)==3&&isvector(inputEdges)&&length(inputEdges)==2
                % weighted average (sum(weighList.*variableList,2)/sum(weighList))
                % parallel axis theorem (sum(weighList.*diag(variableList'*variableList)'))
                obj.pose = inputPose(:)';
                % verts list in body frame (format: [x y z])
                obj.edges = inputEdges(:)';
                obj.edge2body;
                % verts list in world frame
                obj.updateVerts;
            else
                error('Improper input dimension.')
            end
        end
        
        function obj = addDynParam(obj,inputMass)
            inputEdges = obj.edges;
            obj.mass = inputMass;
            obj.inertia = 1/12*inputMass*(inputEdges(2)^2+inputEdges(1)^2);
            obj.inerMat = diag(obj.mass,obj.mass,obj.inertia);
        end
        
        function obj = updatePose(obj,inputPose)
            obj.position = inputPose(:)';
            % regulate angles within [0,2*pi]
            obj.position(3) = mod(obj.position(3),2*pi);
            % update verts
            obj.updateVerts;
        end
        
        function showPose(obj)
            disp('--------Rigid Cuboid--------')
            disp(['pose: ',mat2strf(obj.pose,'%0.2f')])
            disp('----------------------------')
        end   
    end
    
    methods (Access = protected)   
        % display a matrix in a format of %0.mf
        function outStrMat = mat2strf(inNumMat,dispFmt)
            [nrow,ncol] = size(inNumMat);
            outStrMat = '[';
            for i=1:nrow
                for j=1:ncol
                    if j~=ncol
                        outStrMat = [outStrMat,sprintf(dispFmt,inNumMat(i,j)),','];
                    else
                        if nrow~=1
                            outStrMat = [outStrMat,sprintf(dispFmt,inNumMat(i,j)),';'];
                        else
                            outStrMat = [outStrMat,sprintf(dispFmt,inNumMat(i,j))];
                        end
                    end
                end
            end
            outStrMat = [outStrMat,']'];
        end
        
        function obj = updateVerts(obj)       
            % frame update
            obj.frame = SE2(obj.pose);
            % verts position   
            obj.verts = h2e(obj.frame.T*e2h(obj.bverts'))';
        end
        
        function obj = edge2body(obj)
           obj.faces = obj.templateFaces;
           obj.bverts = [obj.templateVerts(:,1)*obj.edges(1)-obj.edges(1)/2,obj.templateVerts(:,2)*obj.edges(2)-obj.edges(2)/2]; 
        end
    end
end