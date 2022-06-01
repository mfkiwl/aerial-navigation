%% Initialize ODE-IC Problem
addpath("utils");
quadrotor_params = struct('M', 1, 'm', 1, 'L', 0.2, 'l', 0.15, 'g', 9.81);
tspan = [0, 100];
init_conds = zeros(16, 1); init_conds(3) = 9.5;
control = @(state, params)controller(state, params);

%% Solve the ODE-IC Problem
[t, sol] = ode45(@(t,y) linear_quadrotor_dynamics(t, y, quadrotor_params, control), tspan, init_conds);

%% Visualize the solution
animate_sol(t, sol, quadrotor_params);

% plot space-frame displacement
f2 = figure(2);
subplot(3,1,1);
plot(t, sol(:,1)); ylabel('x-displacement (m)'); grid on;
subplot(3,1,2);
plot(t, sol(:,2)); ylabel('y-displacement (m)'); grid on;
subplot(3,1,3);
plot(t, sol(:,3)); ylabel('z-displacement (m)'); xlabel('time (s)'); grid on;

% plot space-frame velocity
f3 = figure(3);
subplot(3,1,1);
plot(t, sol(:,4)); ylabel('x-velocity (m/s)'); grid on;
subplot(3,1,2);
plot(t, sol(:,5)); ylabel('y-velocity (m/s)'); grid on;
subplot(3,1,3);
plot(t, sol(:,6)); ylabel('z-velocity (m/s)'); xlabel('time (s)'); grid on;

% plot body-frame yaw-pitch-roll angles
f4 = figure(4);
subplot(3,1,1);
plot(t, sol(:,7)); ylabel('yaw (rad)'); grid on;
subplot(3,1,2);
plot(t, sol(:,8)); ylabel('pitch (rad)'); grid on;
subplot(3,1,3);
plot(t, sol(:,9)); ylabel('roll (rad)'); xlabel('time (s)'); grid on;

% plot body-frame yaw-pitch-roll angle rates
f5 = figure(5);
subplot(3,1,1);
plot(t, sol(:,10)); ylabel('yaw rate (rad/s)'); grid on;
subplot(3,1,2);
plot(t, sol(:,11)); ylabel('pitch rate (rad/s)'); grid on;
subplot(3,1,3);
plot(t, sol(:,12)); ylabel('roll rate (rad/s)'); xlabel('time (s)'); grid on;

% plot control effort
f6 = figure(6);
plot(t, sol(:,13:16)); ylabel('BLDC Angular Velocity [rad/s]'); xlabel('time (s)');
legend('u1', 'u2', 'u3', 'u4'); grid on;