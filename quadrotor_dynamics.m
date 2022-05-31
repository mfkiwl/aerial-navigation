function dx = quadrotor_dynamics(t, state, params)

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
xdot = state(4); % vel. along x-axis w.r.t. some fixed ref frame
ydot = state(5); % vel. along y-axis w.r.t. some fixed ref frame
zdot = state(6); % vel. along z-axis w.r.t. some fixed ref frame
alpha = state(7); % yaw of the quadrotor
beta = state(8); % pitch of the quadrotor
gamma = state(9); % roll of the quadrotor
alphadot = state(10); % yawspeed of the quadrotor
betadot = state(11); % pitchspeed of the quadrotor
gammadot = state(12); % rollspeed of the quadrotor
u1 = state(13); % angular velocity of the first bldc motor
u2 = state(14); % angular velocity of the second bldc motor
u3 = state(15); % angular velocity of the third bldc motor
u4 = state(16); % angular velocity of the fourth bldc motor

% equilibrium angular velocity for stationary flight of quadrotor
u0 = sqrt(1/4 * (M + 4 * m) * g);
u1 = u0;
u2 = -u0;
u3 = u0;
u4 = -u0;

% forward acceleration of the quadrotor (forward meaning upwards if level)
a = 1 / (M + 4 * m) * (u1^2 + u2^2 + u3^2 + u4^2);

% quadrotor dynamics
xddot = a * (cos(alpha) * sin(beta) * cos(gamma) - sin(alpha) * sin(gamma));
yddot = a * (sin(alpha) * sin(beta) * cos(gamma) - sin(alpha) * sin(gamma));
zddot = a * (cos(beta) * cos(gamma)) - g;

alphaddot = m * l / ((M + 4 * m) * L) * (u1 + u2 + u3 + u4);
betaddot = 1 / m * (u1^2 - u3^2);
gammaddot = 1 / m * (u2^2 - u4^2);

% apply to differential to solve
dx = [
    xdot; ydot; zdot; xddot; yddot; zddot;
    alphadot; betadot; gammadot; alphaddot; betaddot; gammaddot;
    0; 0; 0; 0 % no change in 'u's
];

end