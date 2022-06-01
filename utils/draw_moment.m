function pplot = draw_moment(state, params)
% pplot(...) draws a snapshot of the quadrotor using MATLAB plotting
% features.

% Grab the quadrotor parameters
m = params.m;
M = params.M;
l = params.l;
L = params.L;
g = params.g;

% store state vectors in local variables with convenient names
x = state(1); % x-position w.r.t. some fixed point
y = state(2); % y-position w.r.t. some fixed point
z = state(3); % z-position w.r.t. some fixed point
alpha = state(7); % yaw of the quadrotor
beta = state(8); % pitch of the quadrotor
gamma = state(9); % roll of the quadrotor

pplot = plot3(x, y, z, 'o');

end

