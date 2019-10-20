%% create symbolic variables
x = sym('x', [16 1], 'real');
u = sym('omega', [4 1], 'real');
syms t real

% create a symbolic quadrotor parameter structure
quadrotor.nrotors = sym('N', 'real');                %   4 rotors
quadrotor.g = sym('g', 'real');                   %   g       Gravity                             1x1
quadrotor.rho = sym('rho', 'real');                %   rho     Density of air                      1x1
quadrotor.muv = sym('mu_v', 'real');               %   muv     Viscosity of air                    1x1

% Airframe
quadrotor.M = sym('M', 'real');                      %   M       Mass                                1x1
syms Ixx Iyy Izz real
quadrotor.J = diag([Ixx Iyy Izz]);    %   I       Flyer rotational inertia matrix     3x3

quadrotor.h = sym('h', 'real');                 %   h       Height of rotors above CoG          1x1
quadrotor.d = sym('d', 'real');                  %   d       Length of flyer arms                1x1

%Rotor
quadrotor.nb = sym('nb', 'real');                      %   b       Number of blades per rotor          1x1
quadrotor.r = sym('r', 'real');                  %   r       Rotor radius                        1x1

quadrotor.c = sym('c', 'real');                  %   c       Blade chord                         1x1

quadrotor.e = sym('e', 'real');                    %   e       Flapping hinge offset               1x1
quadrotor.Mb = sym('Mb', 'real');                 %   Mb      Rotor blade mass                    1x1
quadrotor.Mc = sym('Mc', 'real');                 %   Mc      Estimated hub clamp mass            1x1
quadrotor.ec = sym('ec', 'real');                 %   ec      Blade root clamp displacement       1x1
quadrotor.Ib = quadrotor.Mb*(quadrotor.r-quadrotor.ec)^2/4 ;        %   Ib      Rotor blade rotational inertia      1x1
quadrotor.Ic = quadrotor.Mc*(quadrotor.ec)^2/4;           %   Ic      Estimated root clamp inertia        1x1
quadrotor.mb = quadrotor.g*(quadrotor.Mc*quadrotor.ec/2+quadrotor.Mb*quadrotor.r/2);    %   mb      Static blade moment                 1x1
quadrotor.Ir = quadrotor.nb*(quadrotor.Ib+quadrotor.Ic);             %   Ir      Total rotor inertia                 1x1

quadrotor.Ct = sym('Ct', 'real');                %   Ct      Non-dim. thrust coefficient         1x1
quadrotor.Cq = quadrotor.Ct*sqrt(quadrotor.Ct/2);         %   Cq      Non-dim. torque coefficient         1x1

quadrotor.sigma = quadrotor.c*quadrotor.nb/(pi*quadrotor.r);         %   sigma   Rotor solidity ratio                1x1
quadrotor.thetat = sym('theta_t', 'real');      %   thetat  Blade tip angle                     1x1
quadrotor.theta0 = sym('theta_0', 'real');     %   theta0  Blade root angle                    1x1
quadrotor.theta1 = quadrotor.thetat - quadrotor.theta0;   %   theta1  Blade twist angle                   1x1
quadrotor.theta75 = quadrotor.theta0 + 0.75*quadrotor.theta1;%   theta76 3/4 blade angle                     1x1
quadrotor.thetai = quadrotor.thetat*(quadrotor.r/quadrotor.e);      %   thetai  Blade ideal root approximation      1x1
quadrotor.a = sym('a', 'real');                    %   a       Lift slope gradient                 1x1

% derived constants
quadrotor.A = pi*quadrotor.r^2;                 %   A       Rotor disc area                     1x1
quadrotor.gamma = quadrotor.rho*quadrotor.a*quadrotor.c*quadrotor.r^4/(quadrotor.Ib+quadrotor.Ic);%   gamma   Lock number                         1x1

quadrotor.b = quadrotor.Ct*quadrotor.rho*quadrotor.A*quadrotor.r^2; % T = b w^2
quadrotor.k = quadrotor.Cq*quadrotor.rho*quadrotor.A*quadrotor.r^3; % Q = k w^2

quadrotor.verbose = false;

quadrotor

%% compute the dynamics symbolically and attempt to simplify (quite slow)
tic

xdot = mdlDerivatives(t, x, u, quadrotor);
xdot = simplify(xdot, 100);

y = mdlOutputs(t,x, quadrotor);
y = simplify(y, 100);
toc

ccode(xdot, 'file', 'xdot.c')
ccode(y, 'file', 'y.c')
%==============================================================
% mdlDerivatives
% Calculate the state derivatives for the next timestep
%==============================================================
%
function sys = mdlDerivatives(t,x,u, quad)
    
    %CONSTANTS
    %Cardinal Direction Indicies
    N = 1;                      %   N       'North'                             1x1
    E = 2;                      %   S       'South'                             1x1
    S = 3;                      %   E       'East'                              1x1
    W = 4;                      %   W       'West'                              1x1
    
    
    D(:,1) = [quad.d;0;quad.h];          %   Di      Rotor hub displacements             1x3
    D(:,2) = [0;quad.d;quad.h];
    D(:,3) = [-quad.d;0;quad.h];
    D(:,4) = [0;-quad.d;quad.h];
    
    %Body-fixed frame references
    e1 = [1;0;0];               %   ei      Body fixed frame references         3x1
    e2 = [0;1;0];
    e3 = [0;0;1];
    
    %EXTRACT ROTOR SPEEDS FROM U
    w = u(1:4);
    
    %EXTRACT STATES FROM X
    z = x(1:3);   % position in {W}
    n = x(4:6);   % RPY angles {W}
    v = x(7:9);   % velocity in {W}
    o = x(10:12); % angular velocity in {W}
    
    %PREPROCESS ROTATION AND WRONSKIAN MATRICIES
    phi = n(1);    % yaw
    the = n(2);    % pitch
    psi = n(3);    % roll
    
    % rotz(phi)*roty(the)*rotx(psi)
    R = [cos(the)*cos(phi) sin(psi)*sin(the)*cos(phi)-cos(psi)*sin(phi) cos(psi)*sin(the)*cos(phi)+sin(psi)*sin(phi);   %BBF > Inertial rotation matrix
         cos(the)*sin(phi) sin(psi)*sin(the)*sin(phi)+cos(psi)*cos(phi) cos(psi)*sin(the)*sin(phi)-sin(psi)*cos(phi);
         -sin(the)         sin(psi)*cos(the)                            cos(psi)*cos(the)];
    
    
    %Manual Construction
    %     Q3 = [cos(phi) -sin(phi) 0;sin(phi) cos(phi) 0;0 0 1];   % RZ %Rotation mappings
    %     Q2 = [cos(the) 0 sin(the);0 1 0;-sin(the) 0 cos(the)];   % RY
    %     Q1 = [1 0 0;0 cos(psi) -sin(psi);0 sin(psi) cos(psi)];   % RX
    %     R = Q3*Q2*Q1    %Rotation matrix
    %
    %    RZ * RY * RX
    iW = [0        sin(psi)          cos(psi);             %inverted Wronskian
          0        cos(psi)*cos(the) -sin(psi)*cos(the);
          cos(the) sin(psi)*sin(the) cos(psi)*sin(the)] / cos(the);
    if any(w == 0)
        % might need to fix this, preculudes aerobatics :(
        % mu becomes NaN due to 0/0
        error('quadrotor_dynamics: not defined for zero rotor speed');
    end
    
    %ROTOR MODEL
    for i=[N E S W] %for each rotor
        %Relative motion
        
        Vr = cross(o,D(:,i)) + v;
        mu = sqrt(sum(Vr(1:2).^2)) / (abs(w(i))*quad.r);  %Magnitude of mu, planar components
        lc = Vr(3) / (abs(w(i))*quad.r);   %Non-dimensionalised normal inflow
        li = mu; %Non-dimensionalised induced velocity approximation
        alphas = atan2(lc,mu);
        j = atan2(Vr(2),Vr(1));  %Sideslip azimuth relative to e1 (zero over nose)
        J = [cos(j) -sin(j);
            sin(j) cos(j)];  %BBF > mu sideslip rotation matrix
        
        %Flapping
        beta = [((8/3*quad.theta0 + 2*quad.theta1)*mu - 2*(lc)*mu)/(1-mu^2/2); %Longitudinal flapping
            0;];%sign(w) * (4/3)*((Ct/sigma)*(2*mu*gamma/3/a)/(1+3*e/2/r) + li)/(1+mu^2/2)]; %Lattitudinal flapping (note sign)
        beta = J'*beta;  %Rotate the beta flapping angles to longitudinal and lateral coordinates.
        a1s(i) = beta(1) - 16/quad.gamma/abs(w(i)) * o(2);
        b1s(i) = beta(2) - 16/quad.gamma/abs(w(i)) * o(1);
        
        %Forces and torques
        T(:,i) = quad.Ct*quad.rho*quad.A*quad.r^2*w(i)^2 * [-cos(b1s(i))*sin(a1s(i)); sin(b1s(i));-cos(a1s(i))*cos(b1s(i))];   %Rotor thrust, linearised angle approximations
        Q(:,i) = -quad.Cq*quad.rho*quad.A*quad.r^3*w(i)*abs(w(i)) * e3;     %Rotor drag torque - note that this preserves w(i) direction sign
        tau(:,i) = cross(T(:,i),D(:,i));    %Torque due to rotor thrust
    end
    
    %RIGID BODY DYNAMIC MODEL
    dz = v;
    dn = iW*o;
    
    dv = quad.g*e3 + R*(1/quad.M)*sum(T,2);
    
    % vehicle can't fall below ground
%     if groundFlag && (z(3) > 0)
%         z(3) = 0;
%         dz(3) = 0;
%     end
    do = inv(quad.J)*(cross(-o,quad.J*o) + sum(tau,2) + sum(Q,2)); %row sum of torques
    sys = [dz;dn;dv;do];   %This is the state derivative vector
end % End of mdlDerivatives.


%==============================================================
% mdlOutputs
% Calculate the output vector for this timestep
%==============================================================
%
function sys = mdlOutputs(t,x, quad)
    
    %TELEMETRY
    if quad.verbose
        disp(sprintf('%0.3f\t',t,x))
    end
    
    % compute output vector as a function of state vector
    %   z      Position                         3x1   (x,y,z)
    %   v      Velocity                         3x1   (xd,yd,zd)
    %   n      Attitude                         3x1   (Y,P,R)
    %   o      Angular velocity                 3x1   (Yd,Pd,Rd)
    
    n = x(4:6);   % RPY angles
    phi = n(1);    % yaw
    the = n(2);    % pitch
    psi = n(3);    % roll
    
    
    % rotz(phi)*roty(the)*rotx(psi)
    R = [cos(the)*cos(phi) sin(psi)*sin(the)*cos(phi)-cos(psi)*sin(phi) cos(psi)*sin(the)*cos(phi)+sin(psi)*sin(phi);   %BBF > Inertial rotation matrix
         cos(the)*sin(phi) sin(psi)*sin(the)*sin(phi)+cos(psi)*cos(phi) cos(psi)*sin(the)*sin(phi)-sin(psi)*cos(phi);
         -sin(the)         sin(psi)*cos(the)                            cos(psi)*cos(the)];
    
    iW = [0        sin(psi)          cos(psi);             %inverted Wronskian
          0        cos(psi)*cos(the) -sin(psi)*cos(the);
          cos(the) sin(psi)*sin(the) cos(psi)*sin(the)] / cos(the);
    
    % return velocity in the body frame
    sys = [ x(1:6);
            inv(R)*x(7:9);   % translational velocity mapped to body frame
            iW*x(10:12)];    % RPY rates mapped to body frame
    %sys = [x(1:6); iW*x(7:9);  iW*x(10:12)];
    %sys = x;
end