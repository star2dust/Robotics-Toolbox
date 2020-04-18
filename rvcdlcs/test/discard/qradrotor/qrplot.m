function h = qrplot(q,s,quad)
u = [q(1),-q(2:3),q(end:-1:4)];
% create an axis
ish = ishold();
if ~ishold
    % if hold is off, set the axis dimensions
    hold on
end

a1s = zeros(1, quad.nrotors);
b1s = zeros(1, quad.nrotors);
% vehicle dimensons
d = quad.d; %Hub displacement from COG
r = quad.r; %Rotor radius

for i = 1:quad.nrotors
    theta = (i-1)/quad.nrotors*2*pi;
    %   Di      Rotor hub displacements (1x3)
    % first rotor is on the x-axis, clockwise order looking down from above
    D(:,i) = [ d*cos(theta); d*sin(theta); 0];
    scal = s;
    %Attitude center displacements
    C(:,i) = [ scal*cos(theta); scal*sin(theta); 0];
end



%READ STATE
z = [u(1);u(2);u(3)];
n = [u(4);u(5);u(6)];

%PREPROCESS ROTATION MATRIX
phi = n(1);    %Euler angles
the = n(2);
psi = n(3);

R = [cos(the)*cos(phi) sin(psi)*sin(the)*cos(phi)-cos(psi)*sin(phi) cos(psi)*sin(the)*cos(phi)+sin(psi)*sin(phi);   %BBF > Inertial rotation matrix
    cos(the)*sin(phi) sin(psi)*sin(the)*sin(phi)+cos(psi)*cos(phi) cos(psi)*sin(the)*sin(phi)-sin(psi)*cos(phi);
    -sin(the)         sin(psi)*cos(the)                            cos(psi)*cos(the)];

%Manual Construction
%Q3 = [cos(psi) -sin(psi) 0;sin(psi) cos(psi) 0;0 0 1];   %Rotation mappings
%Q2 = [cos(the) 0 sin(the);0 1 0;-sin(the) 0 cos(the)];
%Q1 = [1 0 0;0 cos(phi) -sin(phi);0 sin(phi) cos(phi)];
%R = Q3*Q2*Q1;    %Rotation matrix

%CALCULATE FLYER TIP POSITONS USING COORDINATE FRAME ROTATION
F = [1 0 0;0 -1 0;0 0 -1];

%Draw flyer rotors
t = [0:pi/8:2*pi];
for j = 1:length(t)
    circle(:,j) = [r*sin(t(j));r*cos(t(j));0];
end

for i = 1:quad.nrotors
    hub(:,i) = F*(z + R*D(:,i)); %points in the inertial frame
    
    q = 1; %Flapping angle scaling for output display - makes it easier to see what flapping is occurring
    Rr = [cos(q*a1s(i))  sin(q*b1s(i))*sin(q*a1s(i)) cos(q*b1s(i))*sin(q*a1s(i));   %Rotor > Plot frame
        0              cos(q*b1s(i))               -sin(q*b1s(i));
        -sin(q*a1s(i)) sin(q*b1s(i))*cos(q*a1s(i)) cos(q*b1s(i))*cos(q*a1s(i))];
    
    tippath(:,:,i) = F*R*Rr*circle;
    h.rotor(i) = plot3([hub(1,i)+tippath(1,:,i)],[hub(2,i)+tippath(2,:,i)],[hub(3,i)+tippath(3,:,i)],'b-');
end

%Draw flyer
hub0 = F*z;  % centre of vehicle
for i = 1:quad.nrotors
    % line from hub to centre plot3([hub(1,N) hub(1,S)],[hub(2,N) hub(2,S)],[hub(3,N) hub(3,S)],'-b')
    h.line(i) = plot3([hub(1,i) hub0(1)],[hub(2,i) hub0(2)],[hub(3,i) hub0(3)],'-b');
    
    % plot a circle at the hub itself
    h.hub(i) = plot3([hub(1,i)],[hub(2,i)],[hub(3,i)],'o');
end


% restore hold setting
if ~ish
    hold off
end

end