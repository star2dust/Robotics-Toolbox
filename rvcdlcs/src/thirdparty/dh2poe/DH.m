function g = DH (DH_par,ModelVersion)
%DH Homogeneous transformation formulated by a group of DH parameters
%
%   g = DH (DH_par)
%   DH_par:         DH parameters, 1 x 4
%   ModelVersion:   'std'-standard version
%                   'mdf'-modified version
%   g:              Homogeneous transformation, 4 x 4

switch ModelVersion
    
    case 'std'  %standard version

        theta=DH_par(1);
        d=DH_par(2);
        alpha=DH_par(3);
        a=DH_par(4);

        g = [cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha)   a*cos(theta)
            sin(theta)  cos(theta)*cos(alpha)   -cos(theta)*sin(alpha)  a*sin(theta)
            0           sin(alpha)              cos(alpha)              d
            0           0                       0                       1];
        
    case 'mdf'  %modified version

        alpha=DH_par(1);
        a=DH_par(2);        
        theta=DH_par(3);
        d=DH_par(4);


        g = [cos(theta)             -sin(theta)             0               a
            sin(theta)*cos(alpha)   cos(theta)*cos(alpha)   -sin(alpha)     -d*sin(alpha)
            sin(theta)*sin(alpha)   cos(theta)*sin(alpha)   cos(alpha)      d*cos(alpha)
            0                       0                       0               1];
    
    otherwise
        
        error('Illegal model version.')
end