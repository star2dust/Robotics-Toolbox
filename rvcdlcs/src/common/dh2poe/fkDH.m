function g = fkDH (DH_par, H_base, H_tool, q, JointType, ModelVersion)
%fkDH Forward kinematic using DH parameters
%
%   g = fkDH (DH_par, q, JointType)
%   DH_par:         DH parameters, 4 x n
%   H_base:         Homogeneous transformation from base frame to first
%                   link frame
%   H_tool:         Homogeneous transformation from last link frame to tool
%                   frame
%   q:              Joint variables, n x 1
%   JointType:      'R'-revolute joint
%                   'P'-prismatic joint
%   ModelVersion:   'std'-standard version
%                   'mdf'-modified version
%   g:              Homogeneous transformation, 4 x 4

switch ModelVersion
    
    case 'std'

        n = size(DH_par,1);
        g = H_base;

        for i=1:n
            if JointType(i)=='R'
                g=g*DH(DH_par(i,:)+[q(i),0,0,0],'std');
            elseif JointType(i)=='P'
                g=g*DH(DH_par(i,:)+[0,q(i),0,0],'std');
            else
                error('Illegal Joint Type.')
            end
        end

        g = g*H_tool;

    case 'mdf'
        
        n = size(DH_par,1);
        g = H_base;

        for i=1:n
            if JointType(i)=='R'
                g=g*DH(DH_par(i,:)+[0,0,q(i),0],'mdf');
            elseif JointType(i)=='P'
                g=g*DH(DH_par(i,:)+[0,0,0,q(i)],'mdf');
            else
                error('Illegal Joint Type.')
            end
        end

        g = g*H_tool;
        
    otherwise
        
        error('Illegal model version.')
        
end