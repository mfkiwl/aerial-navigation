function animate_sol_helper(t, x, y, z, roll, pitch, yaw, params)
% This Animation code is for QuadCopter. 
% Written by Jitendra Singh. Updated by Zach Bortoff

%% Define design parameters
D2R = pi / 180;
r_p = params.l;   % radius of propeller [meters]
b   = 2 * params.l + 2 * params.L;   % length of total square cover of the whole quadrotor body [meters]
a   = b / 3;   % the legth of small square base of quadcopter (only visual)
H   = 0.06;  % height of drone in Z direction (only visual)
H_m = H+H/2; % height of motor in z direction (only visual)

%% Conversions
ro = 45*D2R; % angle by which rotate the base of quadcopter

Ri = [ % rotation matrix to rotate the coordinates of base
    cos(ro) -sin(ro) 0;
    sin(ro) cos(ro)  0;
    0       0        1];      
base_co = [ % Coordinates of Base
    -a/2  a/2 a/2 -a/2; 
    -a/2 -a/2 a/2 a/2;
    0    0    0    0];
base = Ri*base_co; % rotate base Coordinates by 45 degree 

to = linspace(0, 2*pi);
xp = r_p*cos(to);
yp = r_p*sin(to);
zp = zeros(1,length(to));
%% Define Figure plot
fig1 = figure('pos', [0 50 800 600]);
hg   = gca;
view(50,35);
grid on;
axis equal;
xlim([-3.0 3.0]); ylim([-3.0 3.0]); zlim([-3 3]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
hold(gca, 'on');

%% Design Different parts
% design the base square
drone(1) = patch([base(1,:)],[base(2,:)],[base(3,:)],'r');
drone(2) = patch([base(1,:)],[base(2,:)],[base(3,:)+H],'r');
alpha(drone(1:2),0.7);
% design 2 parpendiculer legs of quadcopter 
[xcylinder ycylinder zcylinder] = cylinder([H/2 H/2]);
drone(3) =  surface(b*zcylinder-b/2,ycylinder,xcylinder+H/2,'facecolor','b');
drone(4) =  surface(ycylinder,b*zcylinder-b/2,xcylinder+H/2,'facecolor','b') ; 
alpha(drone(3:4),0.6);
% design 4 cylindrical motors 
drone(5) = surface(xcylinder+b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
drone(6) = surface(xcylinder-b/2,ycylinder,H_m*zcylinder+H/2,'facecolor','r');
drone(7) = surface(xcylinder,ycylinder+b/2,H_m*zcylinder+H/2,'facecolor','r');
drone(8) = surface(xcylinder,ycylinder-b/2,H_m*zcylinder+H/2,'facecolor','r');
alpha(drone(5:8),0.7);
% design 4 propellers
drone(9)  = patch(xp+b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
drone(10) = patch(xp-b/2,yp,zp+(H_m+H/2),'c','LineWidth',0.5);
drone(11) = patch(xp,yp+b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
drone(12) = patch(xp,yp-b/2,zp+(H_m+H/2),'p','LineWidth',0.5);
alpha(drone(9:12),0.3);

%% create a group object and parent surface
combinedobject = hgtransform('parent',hg );
set(drone,'parent',combinedobject)
 
for i = 1:length(x)
    ba = plot3(x(1:i),y(1:i),z(1:i), 'b:','LineWidth',1.5);

    translation = makehgtform('translate', [x(i) y(i) z(i)]);
    rotation1 = makehgtform('xrotate',(roll(i)));
    rotation2 = makehgtform('yrotate',(pitch(i)));
    rotation3 = makehgtform('zrotate',yaw(i));
    set(combinedobject,'matrix',...
    translation*rotation3*rotation2*rotation1);
    drawnow
    pause(0.01);
end